#include <flatland_plugins/radiation_source.h>
#include <pluginlib/class_list_macros.h>

using namespace flatland_server;

namespace flatland_plugins {

void RadiationSource::OnInitialize(const YAML::Node &config) {
  ParseParameters(config);
  update_timer_.SetRate(update_rate_);
  source_client_ = nh_.serviceClient<flatland_msgs::SpawnRadSource>("spawn_rad_source");
}

void RadiationSource::BeforePhysicsStep(const Timekeeper &timekeeper) {
  // keep the update rate
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  // register the source once
  if (!spawned_ && source_client_.exists()) {
    printf("spawning2\n");
    SpawnSource();
    spawned_ = true;
    printf("spawning3\n");
  }
}

void RadiationSource::SpawnSource() {
  // get world coordinates of source
  b2Body* b2body = body_->physics_body_;
  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();

  // populate service request
  spawn_rad_srv_.request.name = name_;
  spawn_rad_srv_.request.pose.position.x = position.x;
  spawn_rad_srv_.request.pose.position.y = position.y;
  spawn_rad_srv_.request.value = value_;

  //source_client_.waitForExistence();
  if (source_client_.call(spawn_rad_srv_)) {
    printf("spawning\n");
  } else {
    printf("failed\n");
  }
}

void RadiationSource::ParseParameters(const YAML::Node &config) {
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");
  update_rate_ = reader.Get<double>("update_rate", 10.0);
  origin_ = reader.GetPose("origin", Pose(0, 0, 0));
  value_ = reader.Get<double>("value", 0);

  body_ = GetModel()->GetBody(body_name);
  if (!body_) {
    throw YAMLException("Cannot find body with name " + body_name);
  }

  // associate name with model
  name_ = body_->GetName();
}
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::RadiationSource, flatland_server::ModelPlugin)
