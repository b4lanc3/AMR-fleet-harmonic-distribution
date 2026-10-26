// StationArms: gz-sim system animating the 6-axis robot arms that transfer the
// goods between the AMRs and the lines at the line input / output stations.
//
//   input station  (mode="in"):  AMR deck -> transfer conveyor of the line
//   output station (mode="out"): transfer conveyor -> AMR deck
//
// An arm starts a cycle when an AMR stands still on the station's service
// slot.  It is synchronised with the robot through the AMR's cargo lift topic
// (/<amr>/cargo_lift, the robot agent lifts the load on the deck when loading
// is done and lowers it when unloading is done):
//   in : reach down onto the load on the deck, hold it until the AMR has
//        finished unloading (lift goes down), carry it onto the conveyor
//   out: pick the waiting carton on the conveyor, put it down on the deck and
//        hold it until the AMR has finished loading (lift goes up)
//
//   <plugin filename="amr_station_arms" name="amr_gazebo::StationArms">
//     <a1>0.10</a1> <d1>0.30</d1> <l2>0.85</l2> <l3>0.90</l3> <tool>0.19</tool>
//     <carton>0.56 0.40 0.22</carton> <deck_top>0.515</deck_top>
//     <arm name="B1_IN" mode="in">
//       <base>45.25 63.95 0.45</base>          shoulder turret axis, top of the pedestal
//       <service>44.0 65.0</service>           AMR service slot
//       <conveyor>44.0 63.95 0.29 0.0</conveyor> carton place on the belt (x y z yaw)
//     </arm>
//   </plugin>
//
// The links are static visual-only models of the world (arm_<name>_turret,
// _upper, _fore, _tool, _carton, _parcel) posed every iteration by
// inverse kinematics of the tool tip: no physics cost.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace amr_gazebo
{
namespace
{
constexpr double kHidden = -20.0;        // z of hidden cartons
constexpr double kAbove = 0.35;          // approach height above a pick / place
constexpr double kSpeed = 0.9;           // tool tip speed [m/s]
constexpr double kWaitTimeout = 14.0;    // s waiting for the AMR's lift
constexpr double kAtSlot = 0.35;         // AMR on the service slot [m]
constexpr double kStill = 0.02;          // m per check: AMR stands still

double Smooth(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

double WrapPi(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

// tool tip target: cylindrical coordinates around the arm's turret axis
struct Tip
{
  double az{0.0};
  double rho{1.0};
  double z{1.0};
  double yaw{0.0};      // tool (and carried carton) yaw, world frame
};

enum class Wait { None, LiftDown, LiftUp };
enum class Act { None, Grip, Release, Done };

struct Step
{
  Tip to;
  Wait wait{Wait::None};
  Act act{Act::None};
};

struct Arm
{
  std::string name;
  bool in{true};
  gz::math::Vector3d base;
  gz::math::Vector2d service;
  gz::math::Vector3d conv;
  double convYaw{0.0};
  gz::sim::Entity turret{gz::sim::kNullEntity}, upper{gz::sim::kNullEntity},
    fore{gz::sim::kNullEntity}, tool{gz::sim::kNullEntity},
    carton{gz::sim::kNullEntity}, parcel{gz::sim::kNullEntity};

  Tip home, cur, from;
  std::deque<Step> steps;
  double t{0.0};            // time in the current move
  double dur{0.0};
  double waited{0.0};
  bool moving{false};
  bool holding{false};
  bool parcelShown{false};
  double parcelTimer{-1.0}; // s until the parcel on the conveyor appears / disappears
  std::string amr;          // AMR served by the running cycle
  bool busy{false};
  bool served{false};       // cycle done, wait until the AMR leaves the slot
  bool dirty{true};
};

struct Amr
{
  gz::sim::Entity entity{gz::sim::kNullEntity};
  gz::math::Pose3d pose;
  gz::math::Vector2d last;
  double stillFor{0.0};
};

class StationArms
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    auto elem = std::const_pointer_cast<sdf::Element>(_sdf);
    auto get = [&](const char *key, double def) {
      return elem->HasElement(key) ? elem->Get<double>(key) : def;
    };
    this->a1 = get("a1", this->a1);
    this->d1 = get("d1", this->d1);
    this->l2 = get("l2", this->l2);
    this->l3 = get("l3", this->l3);
    this->toolLen = get("tool", this->toolLen);
    this->deckTop = get("deck_top", this->deckTop);
    if (elem->HasElement("carton"))
      this->cartonSize = elem->Get<gz::math::Vector3d>("carton");

    for (auto a = elem->FindElement("arm"); a; a = a->GetNextElement("arm"))
    {
      Arm arm;
      arm.name = a->GetAttribute("name")->GetAsString();
      arm.in = !a->GetAttribute("mode") || a->GetAttribute("mode")->GetAsString() != "out";
      arm.base = a->Get<gz::math::Vector3d>("base");
      arm.service = a->Get<gz::math::Vector2d>("service");
      const auto c = a->Get<gz::math::Pose3d>("conveyor");
      arm.conv = c.Pos();
      arm.convYaw = c.Rot().Yaw();
      // home: above the middle of the conveyor and the AMR slot
      const gz::math::Vector3d mid((arm.conv.X() + arm.service.X()) / 2.0,
                                   (arm.conv.Y() + arm.service.Y()) / 2.0, 1.25);
      arm.home = this->ToTip(arm, mid, arm.convYaw);
      arm.cur = arm.home;
      arm.parcelShown = !arm.in;      // a carton waits on the output conveyor
      this->arms.push_back(arm);
    }
    gzmsg << "[StationArms] " << this->arms.size() << " station arms" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->resolved)
      this->Resolve(_ecm);
    if (_info.paused)
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    const double now = std::chrono::duration<double>(_info.simTime).count();

    if (now - this->lastScan >= 0.1)
    {
      this->ScanAmrs(_ecm, now - this->lastScan, now);
      this->lastScan = now;
      for (auto &arm : this->arms)
        this->Supervise(arm);
    }
    for (auto &arm : this->arms)
    {
      this->Advance(arm, dt);
      if (arm.dirty)
        this->Publish(arm, _ecm);
    }
  }

private:
  // ------------------------------------------------------------- geometry
  Tip ToTip(const Arm &_arm, const gz::math::Vector3d &_p, double _yaw) const
  {
    Tip t;
    const double dx = _p.X() - _arm.base.X(), dy = _p.Y() - _arm.base.Y();
    t.az = std::atan2(dy, dx);
    t.rho = std::hypot(dx, dy);
    t.z = _p.Z();
    t.yaw = _yaw;
    return t;
  }

  gz::math::Vector3d TipPos(const Arm &_arm, const Tip &_t) const
  {
    return {_arm.base.X() + _t.rho * std::cos(_t.az), _arm.base.Y() + _t.rho * std::sin(_t.az),
            _t.z};
  }

  // link poses for a tool tip (wrist above the tip, tool hanging vertically)
  void Links(const Arm &_arm, const Tip &_t, gz::math::Pose3d &_turret, gz::math::Pose3d &_upper,
            gz::math::Pose3d &_fore, gz::math::Pose3d &_tool) const
  {
    const double q1 = _t.az;
    double r = std::max(0.05, _t.rho - this->a1);
    double z = _t.z + this->toolLen - (_arm.base.Z() + this->d1);
    double d = std::hypot(r, z);
    const double dmin = std::abs(this->l2 - this->l3) + 1e-3, dmax = this->l2 + this->l3 - 1e-3;
    const double phi = std::atan2(r, z);          // lean of the shoulder -> wrist line
    d = std::clamp(d, dmin, dmax);
    r = d * std::sin(phi);
    z = d * std::cos(phi);
    const double cosA = (this->l2 * this->l2 + d * d - this->l3 * this->l3) / (2.0 * this->l2 * d);
    const double q2 = phi - std::acos(std::clamp(cosA, -1.0, 1.0));   // elbow up
    const double er = this->l2 * std::sin(q2), ez = this->l2 * std::cos(q2);
    const double q3 = std::atan2(-(z - ez), r - er);                   // forearm down pitch

    const gz::math::Vector3d radial(std::cos(q1), std::sin(q1), 0.0);
    const gz::math::Vector3d s = _arm.base + radial * this->a1 + gz::math::Vector3d(0, 0, this->d1);
    const gz::math::Vector3d e = s + radial * er + gz::math::Vector3d(0, 0, ez);
    const gz::math::Vector3d w = s + radial * r + gz::math::Vector3d(0, 0, z);
    _turret = gz::math::Pose3d(_arm.base, gz::math::Quaterniond(0, 0, q1));
    _upper = gz::math::Pose3d(s, gz::math::Quaterniond(0, q2, q1));
    _fore = gz::math::Pose3d(e, gz::math::Quaterniond(0, q3, q1));
    _tool = gz::math::Pose3d(w, gz::math::Quaterniond(0, 0, _t.yaw));
  }

  // --------------------------------------------------------------- motion
  void Go(Arm &_arm, const gz::math::Vector3d &_p, double _yaw, Wait _wait = Wait::None,
          Act _act = Act::None)
  {
    Step s;
    s.to = this->ToTip(_arm, _p, _yaw);
    s.wait = _wait;
    s.act = _act;
    _arm.steps.push_back(s);
  }

  void Start(Arm &_arm, const std::string &_amr, const gz::math::Pose3d &_amrPose)
  {
    const gz::math::Vector3d up(0, 0, kAbove);
    const double h = this->cartonSize.Z();
    const gz::math::Vector3d deck(_amrPose.Pos().X(), _amrPose.Pos().Y(), this->deckTop);
    const gz::math::Vector3d belt(_arm.conv.X(), _arm.conv.Y(), _arm.conv.Z() + h);
    const double amrYaw = _amrPose.Rot().Yaw();
    _arm.amr = _amr;
    _arm.busy = true;
    _arm.steps.clear();
    if (_arm.in)
    {
      this->Go(_arm, deck + up, amrYaw);
      this->Go(_arm, deck, amrYaw, Wait::LiftDown, Act::Grip);
      this->Go(_arm, deck + up, amrYaw);
      this->Go(_arm, belt + up, _arm.convYaw);
      this->Go(_arm, belt, _arm.convYaw, Wait::None, Act::Release);
      this->Go(_arm, belt + up, _arm.convYaw);
    }
    else
    {
      this->Go(_arm, belt + up, _arm.convYaw);
      this->Go(_arm, belt, _arm.convYaw, Wait::None, Act::Grip);
      this->Go(_arm, belt + up, _arm.convYaw);
      this->Go(_arm, deck + up, amrYaw);
      this->Go(_arm, deck, amrYaw, Wait::LiftUp, Act::Release);
      this->Go(_arm, deck + up, amrYaw);
    }
    Step home;
    home.to = _arm.home;
    home.act = Act::Done;
    _arm.steps.push_back(home);
  }

  void Advance(Arm &_arm, double _dt)
  {
    if (_arm.parcelTimer > 0.0)
    {
      _arm.parcelTimer -= _dt;
      if (_arm.parcelTimer <= 0.0)
      {
        _arm.parcelShown = !_arm.in;     // in: carton conveyed away; out: next carton arrives
        _arm.dirty = true;
      }
    }
    if (_arm.steps.empty())
      return;
    Step &s = _arm.steps.front();
    if (!_arm.moving)
    {
      _arm.from = _arm.cur;
      // shortest swing around the turret; tool yaw modulo pi (the carton is symmetric)
      _arm.from.az = s.to.az - WrapPi(s.to.az - _arm.cur.az);
      _arm.from.yaw = s.to.yaw - (WrapPi(2.0 * (s.to.yaw - _arm.cur.yaw)) / 2.0);
      const double arc = std::hypot(0.5 * (_arm.from.rho + s.to.rho) * (s.to.az - _arm.from.az),
                                    std::hypot(s.to.rho - _arm.from.rho, s.to.z - _arm.from.z));
      _arm.dur = std::max(0.6, arc / kSpeed);
      _arm.t = 0.0;
      _arm.waited = 0.0;
      _arm.moving = true;
    }
    if (_arm.t < _arm.dur)
    {
      _arm.t = std::min(_arm.dur, _arm.t + _dt);
      const double k = Smooth(_arm.t / _arm.dur);
      _arm.cur.az = _arm.from.az + k * (s.to.az - _arm.from.az);
      _arm.cur.rho = _arm.from.rho + k * (s.to.rho - _arm.from.rho);
      _arm.cur.z = _arm.from.z + k * (s.to.z - _arm.from.z);
      _arm.cur.yaw = _arm.from.yaw + k * (s.to.yaw - _arm.from.yaw);
      _arm.dirty = true;
      return;
    }
    if (s.wait != Wait::None)
    {
      _arm.waited += _dt;
      const double lift = this->Lift(_arm.amr);
      const bool ok = s.wait == Wait::LiftDown ? lift < 0.0 : lift > 0.0;
      if (!ok && _arm.waited < kWaitTimeout)
        return;
    }
    switch (s.act)
    {
      case Act::Grip:
        _arm.holding = true;
        if (!_arm.in)
          _arm.parcelShown = false;
        break;
      case Act::Release:
        _arm.holding = false;
        if (_arm.in)
        {
          _arm.parcelShown = true;      // on the belt, conveyed away after a while
          _arm.parcelTimer = 2.5;
        }
        break;
      case Act::Done:
        _arm.busy = false;
        _arm.served = true;
        if (!_arm.in)
          _arm.parcelTimer = 3.0;       // next carton on the output conveyor
        break;
      default:
        break;
    }
    _arm.dirty = true;
    _arm.moving = false;
    _arm.steps.pop_front();
  }

  // -------------------------------------------------------------- AMRs
  double Lift(const std::string &_amr)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    auto it = this->lift.find(_amr);
    return it == this->lift.end() ? 0.0 : it->second;
  }

  void ScanAmrs(gz::sim::EntityComponentManager &_ecm, double _dt, double _now)
  {
    if (_now - this->lastList >= 2.0)
    {
      this->lastList = _now;
      _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_e, const gz::sim::components::Model *,
            const gz::sim::components::Name *_name) -> bool
        {
          const std::string &n = _name->Data();
          if (n.rfind("amr_", 0) == 0 && this->amrs.find(n) == this->amrs.end())
          {
            this->amrs[n].entity = _e;
            std::function<void(const gz::msgs::Double &)> cb =
              [this, n](const gz::msgs::Double &_msg)
              {
                std::lock_guard<std::mutex> lock(this->mutex);
                this->lift[n] = _msg.data();
              };
            this->node.Subscribe("/" + n + "/cargo_lift", cb);
          }
          return true;
        });
    }
    for (auto &[name, amr] : this->amrs)
    {
      auto *pose = _ecm.Component<gz::sim::components::Pose>(amr.entity);
      if (!pose)
        continue;
      amr.pose = pose->Data();
      const gz::math::Vector2d p(amr.pose.Pos().X(), amr.pose.Pos().Y());
      amr.stillFor = (p - amr.last).Length() < kStill ? amr.stillFor + _dt : 0.0;
      amr.last = p;
    }
  }

  void Supervise(Arm &_arm)
  {
    if (_arm.busy)
      return;
    const Amr *at = nullptr;
    std::string who;
    for (const auto &[name, amr] : this->amrs)
    {
      const gz::math::Vector2d p(amr.pose.Pos().X(), amr.pose.Pos().Y());
      if ((p - _arm.service).Length() < kAtSlot)
      {
        at = &amr;
        who = name;
        break;
      }
    }
    if (_arm.served)
    {
      if (at == nullptr || who != _arm.amr)
        _arm.served = false;       // the AMR left: ready for the next one
      return;
    }
    if (at == nullptr || at->stillFor < 0.8 || _arm.parcelTimer > 0.0)
      return;
    const double lift = this->Lift(who);
    if (_arm.in ? lift > 0.0 : (lift < 0.0 && _arm.parcelShown))
      this->Start(_arm, who, at->pose);
  }

  // ------------------------------------------------------------ entities
  void Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    std::map<std::string, gz::sim::Entity> byName;
    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
      [&](const gz::sim::Entity &_e, const gz::sim::components::Model *,
          const gz::sim::components::Name *_name) -> bool
      {
        if (_name->Data().rfind("arm_", 0) == 0)
          byName[_name->Data()] = _e;
        return true;
      });
    auto find = [&](const Arm &_arm, const char *_part) {
      auto it = byName.find("arm_" + _arm.name + "_" + _part);
      return it == byName.end() ? gz::sim::kNullEntity : it->second;
    };
    int missing = 0;
    for (auto &arm : this->arms)
    {
      arm.turret = find(arm, "turret");
      arm.upper = find(arm, "upper");
      arm.fore = find(arm, "fore");
      arm.tool = find(arm, "tool");
      arm.carton = find(arm, "carton");
      arm.parcel = find(arm, "parcel");
      for (auto e : {arm.turret, arm.upper, arm.fore, arm.tool, arm.carton, arm.parcel})
        missing += e == gz::sim::kNullEntity;
    }
    if (missing)
      gzwarn << "[StationArms] " << missing << " arm link models not found" << std::endl;
    this->resolved = true;
  }

  void Set(gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _e, const gz::math::Pose3d &_p)
  {
    if (_e == gz::sim::kNullEntity)
      return;
    auto *pose = _ecm.Component<gz::sim::components::Pose>(_e);
    if (!pose)
      return;
    pose->Data() = _p;
    _ecm.SetChanged(_e, gz::sim::components::Pose::typeId,
                    gz::sim::ComponentState::PeriodicChange);
  }

  void Publish(Arm &_arm, gz::sim::EntityComponentManager &_ecm)
  {
    gz::math::Pose3d turret, upper, fore, tool;
    this->Links(_arm, _arm.cur, turret, upper, fore, tool);
    this->Set(_ecm, _arm.turret, turret);
    this->Set(_ecm, _arm.upper, upper);
    this->Set(_ecm, _arm.fore, fore);
    this->Set(_ecm, _arm.tool, tool);
    const double h = this->cartonSize.Z();
    const gz::math::Vector3d tip = this->TipPos(_arm, _arm.cur);
    this->Set(_ecm, _arm.carton, _arm.holding
      ? gz::math::Pose3d(tip - gz::math::Vector3d(0, 0, h / 2.0),
                         gz::math::Quaterniond(0, 0, _arm.cur.yaw))
      : gz::math::Pose3d(_arm.base.X(), _arm.base.Y(), kHidden, 0, 0, 0));
    this->Set(_ecm, _arm.parcel, _arm.parcelShown
      ? gz::math::Pose3d(_arm.conv + gz::math::Vector3d(0, 0, h / 2.0),
                         gz::math::Quaterniond(0, 0, _arm.convYaw))
      : gz::math::Pose3d(_arm.conv.X(), _arm.conv.Y(), kHidden, 0, 0, 0));
    _arm.dirty = false;
  }

  double a1{0.10}, d1{0.30}, l2{0.85}, l3{0.90}, toolLen{0.19}, deckTop{0.515};
  gz::math::Vector3d cartonSize{0.56, 0.40, 0.22};
  std::vector<Arm> arms;
  std::map<std::string, Amr> amrs;
  std::map<std::string, double> lift;
  std::mutex mutex;
  gz::transport::Node node;
  double lastScan{-1.0};
  double lastList{-10.0};
  bool resolved{false};
};
}  // namespace amr_gazebo

GZ_ADD_PLUGIN(amr_gazebo::StationArms, gz::sim::System,
              amr_gazebo::StationArms::ISystemConfigure,
              amr_gazebo::StationArms::ISystemPreUpdate)
