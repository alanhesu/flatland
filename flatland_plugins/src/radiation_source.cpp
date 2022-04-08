#include <flatland_plugins/radiation_source.h>
#include <flatland_msgs/RadSource.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
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
}

void RadiationSource::GetSource(flatland_msgs::RadSource* source) {
  b2Body* b2body = body_->physics_body_;
  b2Vec2 position = b2body->GetPosition();
  b2Vec2 p1 = b2Vec2(0, 0);
  b2Vec2 p2 = b2Vec2(0, 0);
  if (geometry_ == "line") {
    p1 = b2Vec2(position.x + points_[0].x, position.y + points_[0].y);
    p2 = b2Vec2(position.x + points_[1].x, position.y + points_[1].y);
  }

  source->name = name_;
  source->value = value_;
  source->geometry = geometry_;
  source->pose.position.x = position.x;
  source->pose.position.y = position.y;
  source->p1.x = p1.x;
  source->p1.y = p1.y;
  source->p2.x = p2.x;
  source->p2.y = p2.y;
}

void RadiationSource::SpawnSource() {
  // get world coordinates of source
  return;
  /*
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
  */
}

void RadiationSource::ParseParameters(const YAML::Node &config) {
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");
  update_rate_ = reader.Get<double>("update_rate", 10.0);
  geometry_ = reader.Get<std::string>("geometry", "point");
  origin_ = reader.GetPose("origin", Pose(0, 0, 0));
  value_ = reader.Get<double>("value", 0);
  std::vector<b2Vec2> default_points = {b2Vec2(0, 0), b2Vec2(0, 0)};
  points_ = reader.GetList<b2Vec2>("line_points", default_points, 2, 2);

  body_ = GetModel()->GetBody(body_name);
  if (!body_) {
    throw YAMLException("Cannot find body with name " + body_name);
  }

  // associate name with model
  name_ = body_->GetName();
}
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::RadiationSource, flatland_server::ModelPlugin)
