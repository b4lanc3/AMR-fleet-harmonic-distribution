// LineAnimator: gz-sim system that keeps the production / packaging lines of
// the factory running - product items travel along every conveyor segment in
// an endless, seamless loop.  Each line can be stopped / started (or slowed
// down) from the MES through a gz topic carrying "B1=1;B2=0;C3=0.5".
//
//   <plugin filename="amr_line_animator" name="amr_gazebo::LineAnimator">
//     <topic>/factory/line_state</topic>
//     <segment line="B1" name="B1_s0">
//       <start>44 63.3 1.0</start> <end>44 55.9 1.0</end>
//       <size>0.22 0.22 0.12</size> <color>0.55 0.6 0.68 1</color>
//       <spacing>0.9</spacing> <speed>0.22</speed>
//     </segment>
//     ...
//   </plugin>
//
// The items are visual-only static models whose pose is updated every
// iteration, so they cost no physics.

#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Root.hh>

namespace amr_gazebo
{
struct Segment
{
  std::string line;
  std::string name;
  gz::math::Vector3d start;
  gz::math::Vector3d end;
  gz::math::Vector3d size{0.3, 0.3, 0.2};
  gz::math::Color color{0.74, 0.56, 0.34, 1.0};
  double spacing{1.0};
  double speed{0.2};
  double offset{0.0};
  std::vector<gz::sim::Entity> items;
};

class LineAnimator
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
    std::string topic = "/factory/line_state";
    if (_sdf->HasElement("topic"))
      topic = _sdf->Get<std::string>("topic");

    auto elem = std::const_pointer_cast<sdf::Element>(_sdf);
    for (auto s = elem->FindElement("segment"); s; s = s->GetNextElement("segment"))
    {
      Segment seg;
      seg.line = s->GetAttribute("line") ? s->GetAttribute("line")->GetAsString() : "";
      seg.name = s->GetAttribute("name") ? s->GetAttribute("name")->GetAsString()
                                         : seg.line + std::to_string(this->segments.size());
      seg.start = s->Get<gz::math::Vector3d>("start");
      seg.end = s->Get<gz::math::Vector3d>("end");
      if (s->HasElement("size"))
        seg.size = s->Get<gz::math::Vector3d>("size");
      if (s->HasElement("color"))
        seg.color = s->Get<gz::math::Color>("color");
      if (s->HasElement("spacing"))
        seg.spacing = std::max(0.2, s->Get<double>("spacing"));
      if (s->HasElement("speed"))
        seg.speed = s->Get<double>("speed");
      this->segments.push_back(seg);
    }
    this->node.Subscribe(topic, &LineAnimator::OnLineState, this);
    gzmsg << "[LineAnimator] " << this->segments.size() << " conveyor segments, line state on ["
          << topic << "]" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->created)
    {
      this->CreateItems(_ecm);
      this->created = true;
    }
    if (_info.paused)
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    std::map<std::string, double> factors;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      factors = this->lineFactor;
    }
    for (auto &seg : this->segments)
    {
      auto it = factors.find(seg.line);
      const double f = it == factors.end() ? 1.0 : it->second;
      if (f <= 0.0 && seg.offset >= 0.0 && this->initialised)
        continue;   // conveyor stopped: items stay where they are
      seg.offset = std::fmod(seg.offset + seg.speed * f * dt, seg.spacing);
      const gz::math::Vector3d dir = (seg.end - seg.start);
      const double length = dir.Length();
      if (length < 1e-6)
        continue;
      const gz::math::Vector3d u = dir / length;
      const double yaw = std::atan2(u.Y(), u.X());
      for (std::size_t k = 0; k < seg.items.size(); ++k)
      {
        const double s = seg.offset + static_cast<double>(k) * seg.spacing;
        const gz::math::Vector3d p = seg.start + u * std::min(s, length);
        auto *pose = _ecm.Component<gz::sim::components::Pose>(seg.items[k]);
        if (!pose)
          continue;
        pose->Data() = gz::math::Pose3d(p, gz::math::Quaterniond(0, 0, yaw));
        _ecm.SetChanged(seg.items[k], gz::sim::components::Pose::typeId,
                        gz::sim::ComponentState::PeriodicChange);
      }
    }
    this->initialised = true;
  }

private:
  void OnLineState(const gz::msgs::StringMsg &_msg)
  {
    // "B1=1;B2=0;C1=0.5"
    std::lock_guard<std::mutex> lock(this->mutex);
    std::stringstream ss(_msg.data());
    std::string item;
    while (std::getline(ss, item, ';'))
    {
      const auto eq = item.find('=');
      if (eq == std::string::npos)
        continue;
      try
      {
        this->lineFactor[item.substr(0, eq)] = std::stod(item.substr(eq + 1));
      }
      catch (...)
      {
      }
    }
  }

  void CreateItems(gz::sim::EntityComponentManager &_ecm)
  {
    gz::sim::SdfEntityCreator creator(_ecm, *this->eventMgr);
    std::size_t total = 0;
    for (auto &seg : this->segments)
    {
      const double length = (seg.end - seg.start).Length();
      const int n = static_cast<int>(std::floor(length / seg.spacing));
      for (int k = 0; k < n; ++k)
      {
        std::ostringstream sdfText;
        const auto &c = seg.color;
        sdfText << "<?xml version='1.0'?><sdf version='1.9'><model name='product_" << seg.name
                << "_" << k << "'><static>true</static><link name='link'><visual name='v'>"
                << "<geometry><box><size>" << seg.size.X() << " " << seg.size.Y() << " "
                << seg.size.Z() << "</size></box></geometry><material><ambient>" << c.R() << " "
                << c.G() << " " << c.B() << " " << c.A() << "</ambient><diffuse>" << c.R() << " "
                << c.G() << " " << c.B() << " " << c.A()
                << "</diffuse></material><cast_shadows>false</cast_shadows></visual></link>"
                << "</model></sdf>";
        sdf::Root root;
        auto errors = root.LoadSdfString(sdfText.str());
        if (!errors.empty() || !root.Model())
        {
          gzerr << "[LineAnimator] cannot create item for " << seg.name << std::endl;
          continue;
        }
        const gz::sim::Entity e = creator.CreateEntities(root.Model());
        creator.SetParent(e, this->world);
        auto *pose = _ecm.Component<gz::sim::components::Pose>(e);
        if (pose)
          pose->Data() = gz::math::Pose3d(seg.start, gz::math::Quaterniond::Identity);
        seg.items.push_back(e);
        ++total;
      }
    }
    gzmsg << "[LineAnimator] created " << total << " product items" << std::endl;
  }

  gz::sim::Entity world{gz::sim::kNullEntity};
  gz::sim::EventManager *eventMgr{nullptr};
  std::vector<Segment> segments;
  std::map<std::string, double> lineFactor;
  std::mutex mutex;
  gz::transport::Node node;
  bool created{false};
  bool initialised{false};
};
}  // namespace amr_gazebo

GZ_ADD_PLUGIN(amr_gazebo::LineAnimator, gz::sim::System,
              amr_gazebo::LineAnimator::ISystemConfigure,
              amr_gazebo::LineAnimator::ISystemPreUpdate)
