#include <pluginlib/class_list_macros.h>
#include <flatland_plugins/radiation_source_world.h>
#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/world_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <geometry_msgs/Pose.h>
#include <boost/algorithm/string/join.hpp>
#include <cmath>
#include <limits>

using namespace flatland_server;

namespace flatland_plugins {

void RadiationSourceWorld::OnInitialize(const YAML::Node &config) {
  ParseParameters(config);

  update_timer_.SetRate(update_rate_);
  source_publisher_ = nh_.advertise<flatland_msgs::RadSources>(topic_, 50);

  spawn_source_service_ = nh_.advertiseService("spawn_rad_source", &RadiationSourceWorld::SpawnRadSource, this);

  if (spawn_source_service_) {
    ROS_INFO_NAMED("Service Manager", "Radiation source spawning service ready to go");
  } else {
    ROS_ERROR_NAMED("Service Manager", "Error starting radiation source spawning service");
  }
}

void RadiationSourceWorld::BeforePhysicsStep(const Timekeeper &timekeeper) {
  // keep the update rate
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  PopulateSourceMsg(timekeeper);

  // only compute and publish when the number of subscribers is not zero
  if (source_publisher_.getNumSubscribers() > 0) {
    source_publisher_.publish(source_msg_);
  }
}

bool RadiationSourceWorld::SpawnRadSource(flatland_msgs::SpawnRadSource::Request &request, flatland_msgs::SpawnRadSource::Response &response) {
  //TODO: add debug printout
  printf("spawned\n");
  std::string name = request.name;
  geometry_msgs::Pose pose = request.pose;
  float value = request.value;

  if (source_size_ >= source_max_) {
    int new_max = source_max_*2;
    Source* sources_new = new Source[new_max];
    for (int i = 0; i < source_size_; i++) {
      sources_new[i] = sources_[i];
    }
    delete[] sources_;
    sources_ = sources_new;
    source_max_ = new_max;
  }
  sources_[source_size_].name = name;
  sources_[source_size_].x = pose.position.x;
  sources_[source_size_].y = pose.position.y;
  sources_[source_size_].val = value;

  response.success = true;
  response.message = sources_[source_size_].name;

  source_size_++;
  return true;
}

void RadiationSourceWorld::PopulateSourceMsg(const Timekeeper &timekeeper) {
  source_msg_.header.stamp = timekeeper.GetSimTime();
  std::vector<std::string> names;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> val;
  for (int i = 0; i < source_size_; i++) {
    names.push_back(sources_[i].name);
    x.push_back(sources_[i].x);
    y.push_back(sources_[i].y);
    val.push_back(sources_[i].val);
  }
  source_msg_.names = names;
  source_msg_.x = x;
  source_msg_.y = y;
  source_msg_.values = val;
}

void RadiationSourceWorld::ParseParameters(const YAML::Node &config) {
  YamlReader reader(config);
  topic_ = reader.Get<std::string>("topic", "rad_sources");
  update_rate_ = reader.Get<double>("update_rate",
                                    std::numeric_limits<double>::infinity());

  std::vector<std::string> layers =
      reader.GetList<std::string>("layers", {"all"}, -1, -1);

  reader.EnsureAccessedAllKeys();

  // init the random number generators
  std::random_device rd;
  rng_ = std::default_random_engine(rd());

  //ROS_DEBUG_NAMED("RadiationSourceWorldPlugin",
                  //"RadiationSourceWorld %s params: topic(%s) body(%s, %p) origin(%f,%f,%f) "
                  //"frame_id(%s) broadcast_tf(%d) update_rate(%f) range(%f)  "
                  //"noise_std_dev(%f) value(%f) layers(0x%u {%s})",
                  //GetName().c_str(), topic_.c_str(), body_name.c_str(), body_,
                  //origin_.x, origin_.y, origin_.theta, frame_id_.c_str(),
                  //broadcast_tf_, update_rate_, range_, noise_std_dev_,
                  //rad_val_, layers_bits_,
                  //boost::algorithm::join(layers, ",").c_str());
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::RadiationSourceWorld, flatland_server::WorldPlugin)
