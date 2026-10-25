// Pedestrians: gz-sim system with factory workers walking along their routes
// (walkways between racks and lines, zebra crossings over the robot lanes).
//
//   <plugin filename="amr_pedestrians" name="amr_gazebo::Pedestrians">
//     <topic>/factory/people</topic>          <!-- gz.msgs.Pose_V, 10 Hz -->
//     <robot_prefix>amr_</robot_prefix>
//     <stop_distance>2.0</stop_distance>
//     <person name="picker_A" speed="1.0" loop="false" vest="1 0.45 0 1" helmet="1 1 1 1">
//       <waypoint dwell="4">12 44.25</waypoint>
//       <waypoint>32 44.25</waypoint>
//       ...
//     </person>
//   </plugin>
//
// Every worker is built from five visual-only static models (body, two legs,
// two arms) whose poses are updated at 50 Hz: legs and arms swing with the
// walking speed.  The legs cross the 2D lidar plane and the whole body is seen
// by the RGB-D camera, so the AMRs' Nav2 collision monitor slows down / stops
// for them like for a real person.  A worker waits while a moving AMR is close
// ahead and turns back if an AMR stands on the walkway for a long time.

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Root.hh>

namespace amr_gazebo
{
namespace
{
constexpr double kHip = 0.92;        // m, hip joint height
constexpr double kShoulder = 1.43;   // m
constexpr double kStride = 1.3;      // m walked per full leg cycle
constexpr double kSwing = 0.42;      // rad, leg swing amplitude
constexpr double kTurnRate = 4.0;    // rad/s
constexpr double kUpdatePeriod = 0.02;

std::string Rgba(const gz::math::Color &_c)
{
  std::ostringstream s;
  s << _c.R() << " " << _c.G() << " " << _c.B() << " " << _c.A();
  return s.str();
}

std::string Box(const std::string &_name, double _x, double _y, double _z,
                double _sx, double _sy, double _sz, const gz::math::Color &_c)
{
  std::ostringstream s;
  s << "<visual name='" << _name << "'><pose>" << _x << " " << _y << " " << _z
    << " 0 0 0</pose><geometry><box><size>" << _sx << " " << _sy << " " << _sz
    << "</size></box></geometry><material><ambient>" << Rgba(_c) << "</ambient><diffuse>"
    << Rgba(_c) << "</diffuse></material><cast_shadows>false</cast_shadows></visual>";
  return s.str();
}

std::string Sphere(const std::string &_name, double _x, double _y, double _z, double _r,
                   const gz::math::Color &_c)
{
  std::ostringstream s;
  s << "<visual name='" << _name << "'><pose>" << _x << " " << _y << " " << _z
    << " 0 0 0</pose><geometry><sphere><radius>" << _r << "</radius></sphere></geometry>"
    << "<material><ambient>" << Rgba(_c) << "</ambient><diffuse>" << Rgba(_c)
    << "</diffuse></material><cast_shadows>false</cast_shadows></visual>";
  return s.str();
}

std::string Cylinder(const std::string &_name, double _x, double _y, double _z, double _r,
                     double _l, const gz::math::Color &_c)
{
  std::ostringstream s;
  s << "<visual name='" << _name << "'><pose>" << _x << " " << _y << " " << _z
    << " 0 0 0</pose><geometry><cylinder><radius>" << _r << "</radius><length>" << _l
    << "</length></cylinder></geometry><material><ambient>" << Rgba(_c) << "</ambient><diffuse>"
    << Rgba(_c) << "</diffuse></material><cast_shadows>false</cast_shadows></visual>";
  return s.str();
}

double Wrap(double _a)
{
  while (_a > GZ_PI) _a -= 2 * GZ_PI;
  while (_a < -GZ_PI) _a += 2 * GZ_PI;
  return _a;
}
}  // namespace

struct Person
{
  std::string name;
  double speed{1.0};
  bool loop{false};
  gz::math::Color vest{1.0, 0.45, 0.0, 1.0};
  gz::math::Color helmet{1.0, 1.0, 1.0, 1.0};
  std::vector<gz::math::Vector2d> waypoints;
  std::vector<double> dwell;

  gz::math::Vector2d pos;
  double heading{0.0};
  double phase{0.0};
  double dwellLeft{0.0};
  double blocked{0.0};
  std::size_t target{1};
  int dir{1};
  bool walking{false};

  gz::sim::Entity body{gz::sim::kNullEntity};
  gz::sim::Entity legL{gz::sim::kNullEntity}, legR{gz::sim::kNullEntity};
  gz::sim::Entity armL{gz::sim::kNullEntity}, armR{gz::sim::kNullEntity};
};

struct RobotTrack
{
  gz::math::Vector2d pos;
  double speed{0.0};
  double stamp{-1.0};
};

class Pedestrians
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &_eventMgr) override
  {
    this->world = _entity;
    this->eventMgr = &_eventMgr;
    std::string topic = "/factory/people";
    if (_sdf->HasElement("topic"))
      topic = _sdf->Get<std::string>("topic");
    if (_sdf->HasElement("robot_prefix"))
      this->robotPrefix = _sdf->Get<std::string>("robot_prefix");
    if (_sdf->HasElement("stop_distance"))
      this->stopDistance = _sdf->Get<double>("stop_distance");

    auto elem = std::const_pointer_cast<sdf::Element>(_sdf);
    for (auto p = elem->FindElement("person"); p; p = p->GetNextElement("person"))
    {
      Person person;
      auto attr = [&p](const std::string &_k) {
        return p->GetAttribute(_k) ? p->GetAttribute(_k)->GetAsString() : std::string();
      };
      person.name = attr("name").empty() ? "worker_" + std::to_string(this->people.size())
                                         : attr("name");
      if (!attr("speed").empty())
        person.speed = std::stod(attr("speed"));
      person.loop = attr("loop") == "true" || attr("loop") == "1";
      if (!attr("vest").empty())
        std::istringstream(attr("vest")) >> person.vest;
      if (!attr("helmet").empty())
        std::istringstream(attr("helmet")) >> person.helmet;
      for (auto w = p->FindElement("waypoint"); w; w = w->GetNextElement("waypoint"))
      {
        person.waypoints.push_back(w->Get<gz::math::Vector2d>());
        double d = 0.0;
        if (w->GetAttribute("dwell"))
          d = std::stod(w->GetAttribute("dwell")->GetAsString());
        person.dwell.push_back(d);
      }
      if (person.waypoints.size() < 2)
      {
        gzerr << "[Pedestrians] " << person.name << " needs at least 2 waypoints" << std::endl;
        continue;
      }
      person.pos = person.waypoints[0];
      const auto d = person.waypoints[1] - person.waypoints[0];
      person.heading = std::atan2(d.Y(), d.X());
      person.phase = 0.37 * static_cast<double>(this->people.size());
      this->people.push_back(person);
    }
    this->pub = this->node.Advertise<gz::msgs::Pose_V>(topic);
    gzmsg << "[Pedestrians] " << this->people.size() << " workers, poses on [" << topic << "]"
          << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->created)
    {
      for (auto &p : this->people)
        this->CreatePerson(_ecm, p);
      this->created = true;
    }
    if (_info.paused)
      return;
    const double now = std::chrono::duration<double>(_info.simTime).count();
    this->accum += std::chrono::duration<double>(_info.dt).count();
    if (this->accum < kUpdatePeriod)
      return;
    const double dt = std::min(this->accum, 0.2);
    this->accum = 0.0;

    this->TrackRobots(_ecm, now);
    for (auto &p : this->people)
    {
      this->Step(p, dt);
      this->Apply(_ecm, p);
    }
    if (now - this->lastPublish >= 0.1)
    {
      this->lastPublish = now;
      gz::msgs::Pose_V msg;
      auto *frame = msg.mutable_header()->add_data();   // -> header.frame_id in ROS
      frame->set_key("frame_id");
      frame->add_value("map");
      msg.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(now));
      msg.mutable_header()->mutable_stamp()->set_nsec(
        static_cast<int32_t>((now - std::floor(now)) * 1e9));
      for (const auto &p : this->people)
      {
        auto *pose = msg.add_pose();
        pose->set_name(p.name);
        pose->mutable_position()->set_x(p.pos.X());
        pose->mutable_position()->set_y(p.pos.Y());
        pose->mutable_orientation()->set_z(std::sin(p.heading / 2));
        pose->mutable_orientation()->set_w(std::cos(p.heading / 2));
      }
      this->pub.Publish(msg);
    }
  }

private:
  void TrackRobots(gz::sim::EntityComponentManager &_ecm, double _now)
  {
    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name,
              gz::sim::components::Pose, gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &_e, const gz::sim::components::Model *,
          const gz::sim::components::Name *_name, const gz::sim::components::Pose *_pose,
          const gz::sim::components::ParentEntity *_parent) -> bool
      {
        if (_parent->Data() != this->world ||
            _name->Data().rfind(this->robotPrefix, 0) != 0)
          return true;
        const gz::math::Vector2d xy(_pose->Data().Pos().X(), _pose->Data().Pos().Y());
        auto &t = this->robots[_e];
        if (t.stamp >= 0.0 && _now > t.stamp)
          t.speed = 0.7 * t.speed + 0.3 * (xy - t.pos).Length() / (_now - t.stamp);
        t.pos = xy;
        t.stamp = _now;
        return true;
      });
  }

  // a moving AMR close ahead, or an AMR standing on the walkway
  bool Blocked(const Person &_p, const gz::math::Vector2d &_dir) const
  {
    for (const auto &kv : this->robots)
    {
      const auto rel = kv.second.pos - _p.pos;
      const double ahead = rel.Dot(_dir);
      const double lateral = std::abs(rel.X() * _dir.Y() - rel.Y() * _dir.X());
      if (ahead <= 0.0)
        continue;
      if (kv.second.speed > 0.05 && ahead < this->stopDistance && lateral < 1.2)
        return true;
      if (ahead < 1.3 && lateral < 0.8)
        return true;
    }
    return false;
  }

  void Step(Person &_p, double _dt)
  {
    _p.walking = false;
    if (_p.dwellLeft > 0.0)
    {
      _p.dwellLeft -= _dt;
      return;
    }
    const auto goal = _p.waypoints[_p.target];
    auto d = goal - _p.pos;
    const double dist = d.Length();
    if (dist < 0.05)
    {
      this->Arrive(_p);
      return;
    }
    d /= dist;
    const double err = Wrap(std::atan2(d.Y(), d.X()) - _p.heading);
    _p.heading = Wrap(_p.heading + std::clamp(err, -kTurnRate * _dt, kTurnRate * _dt));
    if (std::abs(err) > 0.6)
      return;   // turn on the spot first
    if (this->Blocked(_p, d))
    {
      _p.blocked += _dt;
      if (_p.blocked > 8.0)
      {
        // an AMR keeps standing on the walkway: walk back the way we came
        _p.blocked = 0.0;
        _p.dir = -_p.dir;
        this->NextTarget(_p);
      }
      return;
    }
    _p.blocked = 0.0;
    const double step = std::min(_p.speed * _dt, dist);
    _p.pos += d * step;
    _p.phase += step / kStride;
    _p.walking = true;
  }

  void Arrive(Person &_p)
  {
    _p.pos = _p.waypoints[_p.target];
    _p.dwellLeft = _p.dwell[_p.target];
    this->NextTarget(_p);
  }

  void NextTarget(Person &_p)
  {
    const long n = static_cast<long>(_p.waypoints.size());
    long t = static_cast<long>(_p.target) + _p.dir;
    if (_p.loop)
    {
      t = ((t % n) + n) % n;
    }
    else if (t < 0 || t >= n)
    {
      _p.dir = -_p.dir;
      t = static_cast<long>(_p.target) + _p.dir;
    }
    _p.target = static_cast<std::size_t>(std::clamp(t, 0L, n - 1));
  }

  void Apply(gz::sim::EntityComponentManager &_ecm, const Person &_p)
  {
    const double c = std::cos(_p.heading), s = std::sin(_p.heading);
    auto at = [&](double _lx, double _ly, double _z) {
      return gz::math::Vector3d(_p.pos.X() + c * _lx - s * _ly, _p.pos.Y() + s * _lx + c * _ly, _z);
    };
    // legs swing while walking, return to standing when stopped
    const double swing = _p.walking ? kSwing * std::sin(2 * GZ_PI * _p.phase) : 0.0;
    auto set = [&](gz::sim::Entity _e, const gz::math::Vector3d &_pos, double _pitch) {
      auto *pose = _ecm.Component<gz::sim::components::Pose>(_e);
      if (!pose)
        return;
      pose->Data() = gz::math::Pose3d(_pos, gz::math::Quaterniond(0, _pitch, _p.heading));
      _ecm.SetChanged(_e, gz::sim::components::Pose::typeId,
                      gz::sim::ComponentState::PeriodicChange);
    };
    set(_p.body, at(0, 0, 0), 0.0);
    set(_p.legL, at(0, 0.10, kHip), swing);
    set(_p.legR, at(0, -0.10, kHip), -swing);
    set(_p.armL, at(0, 0.25, kShoulder), -0.8 * swing);
    set(_p.armR, at(0, -0.25, kShoulder), 0.8 * swing);
  }

  gz::sim::Entity Spawn(gz::sim::EntityComponentManager &_ecm, const std::string &_name,
                        const std::string &_visuals)
  {
    const std::string text = "<?xml version='1.0'?><sdf version='1.9'><model name='" + _name +
                             "'><static>true</static><link name='link'>" + _visuals +
                             "</link></model></sdf>";
    sdf::Root root;
    auto errors = root.LoadSdfString(text);
    if (!errors.empty() || !root.Model())
    {
      gzerr << "[Pedestrians] cannot create " << _name << std::endl;
      return gz::sim::kNullEntity;
    }
    gz::sim::SdfEntityCreator creator(_ecm, *this->eventMgr);
    const gz::sim::Entity e = creator.CreateEntities(root.Model());
    creator.SetParent(e, this->world);
    return e;
  }

  void CreatePerson(gz::sim::EntityComponentManager &_ecm, Person &_p)
  {
    const gz::math::Color trousers(0.12, 0.16, 0.26, 1), boots(0.08, 0.07, 0.06, 1);
    const gz::math::Color shirt(0.20, 0.30, 0.45, 1), skin(0.87, 0.70, 0.56, 1);
    const gz::math::Color stripe(0.85, 0.87, 0.88, 1);
    std::string body =
      Box("pelvis", 0, 0, kHip + 0.04, 0.20, 0.34, 0.14, trousers) +
      Box("torso", 0, 0, 1.19, 0.24, 0.42, 0.52, _p.vest) +
      Box("stripe1", 0, 0, 1.08, 0.245, 0.425, 0.04, stripe) +
      Box("stripe2", 0, 0, 1.28, 0.245, 0.425, 0.04, stripe) +
      Box("shoulders", 0, 0, 1.42, 0.22, 0.48, 0.07, shirt) +
      Cylinder("neck", 0, 0, 1.49, 0.05, 0.08, skin) +
      Sphere("head", 0.01, 0, 1.62, 0.105, skin) +
      Sphere("helmet", 0, 0, 1.67, 0.12, _p.helmet) +
      Box("brim", 0.10, 0, 1.655, 0.10, 0.20, 0.015, _p.helmet);
    std::string leg = Box("leg", 0, 0, -0.42, 0.13, 0.15, 0.84, trousers) +
                      Box("boot", 0.04, 0, -0.87, 0.26, 0.13, 0.10, boots);
    std::string arm = Box("sleeve", 0, 0, -0.27, 0.09, 0.09, 0.54, shirt) +
                      Box("hand", 0, 0, -0.58, 0.07, 0.08, 0.09, skin);
    _p.body = this->Spawn(_ecm, _p.name, body);
    _p.legL = this->Spawn(_ecm, _p.name + "_leg_l", leg);
    _p.legR = this->Spawn(_ecm, _p.name + "_leg_r", leg);
    _p.armL = this->Spawn(_ecm, _p.name + "_arm_l", arm);
    _p.armR = this->Spawn(_ecm, _p.name + "_arm_r", arm);
    this->Apply(_ecm, _p);
  }

  gz::sim::Entity world{gz::sim::kNullEntity};
  gz::sim::EventManager *eventMgr{nullptr};
  std::vector<Person> people;
  std::map<gz::sim::Entity, RobotTrack> robots;
  std::string robotPrefix{"amr_"};
  double stopDistance{2.0};
  double accum{0.0};
  double lastPublish{-1.0};
  bool created{false};
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
};
}  // namespace amr_gazebo

GZ_ADD_PLUGIN(amr_gazebo::Pedestrians, gz::sim::System,
              amr_gazebo::Pedestrians::ISystemConfigure,
              amr_gazebo::Pedestrians::ISystemPreUpdate)
