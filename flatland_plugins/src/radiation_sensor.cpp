#include <flatland_plugins/radiation_sensor.h>
#include <flatland_msgs/RadSources.h>
#include <flatland_msgs/RadSource.h>
#include <flatland_msgs/RadSensor.h>
#include <pluginlib/class_list_macros.h>
#include <math.h>

using namespace flatland_server;

namespace flatland_plugins {

void RadiationSensor::SourceCallback(const flatland_msgs::RadSources& msg) {
  source_msg_ = msg;
}

void RadiationSensor::UpdateSource(const Timekeeper &timekeeper) {
  const b2Transform &t = body_->GetPhysicsBody()->GetTransform();
  Eigen::Matrix3f m_world_to_body;
  m_world_to_body << t.q.c, -t.q.s, t.p.x, t.q.s, t.q.c, t.p.y, 0, 0, 1;
  Eigen::Matrix3f m_world_to_sensor = m_world_to_body * m_body_to_sensor_;

  b2Vec2 sensor_pos(m_world_to_sensor(0, 2), m_world_to_sensor(1, 2));

  //TODO: add angle sensitivity

  // sum up contribution from sources
  float sensor_val = 0.0;
  for (flatland_msgs::RadSource source : source_msg_.sources) {
    sensor_val += CalcSource(sensor_pos, source);
  }

  // construct sensor message
  sensor_msg_.header.stamp = timekeeper.GetSimTime();
  sensor_msg_.value = sensor_val;
}

void RadiationSensor::OnInitialize(const YAML::Node &config) {
  ParseParameters(config);
  update_timer_.SetRate(update_rate_);
  sensor_publisher_ = nh_.advertise<flatland_msgs::RadSensor>(topic_, 1);
  source_subscriber_ = nh_.subscribe(source_topic_, 1, &RadiationSensor::SourceCallback, this);

  double c = cos(origin_.theta);
  double s = sin(origin_.theta);
  double x = origin_.x, y = origin_.y;
  m_body_to_sensor_ << c, -s, x, s, c, y, 0, 0, 1;
}

void RadiationSensor::BeforePhysicsStep(const Timekeeper &timekeeper) {
  // keep the update rate
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  // only compute and publish when the number of subscribers is not zero
  if (sensor_publisher_.getNumSubscribers() > 0) {
    UpdateSource(timekeeper);
    sensor_publisher_.publish(sensor_msg_);
  }

  if (broadcast_tf_) {
    sensor_tf_.header.stamp = timekeeper.GetSimTime();
    tf_broadcaster_.sendTransform(sensor_tf_);
  }
}

void RadiationSensor::ParseParameters(const YAML::Node &config) {
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");
  topic_ = reader.Get<std::string>("topic", "rad_sensor");
  source_topic_ = reader.Get<std::string>("sub_topic", "rad_sources");
  frame_id_ = reader.Get<std::string>("frame", GetName());
  broadcast_tf_ = reader.Get<bool>("broadcast_tf", true);
  update_rate_ = reader.Get<double>("update_rate", 10.0);
  origin_ = reader.GetPose("origin", Pose(0, 0, 0));

  body_ = GetModel()->GetBody(body_name);
  if (!body_) {
    throw YAMLException("Cannot find body with name " + body_name);
  }

  std::string parent_frame_id =
      tf::resolve("", GetModel()->NameSpaceTF(body_->GetName()));
  std::string child_frame_id =
      tf::resolve("", GetModel()->NameSpaceTF(frame_id_));

  sensor_msg_.header.frame_id = child_frame_id;

  // Construct constant TF transform
  sensor_tf_.header.frame_id = parent_frame_id;
  sensor_tf_.child_frame_id = child_frame_id;
  sensor_tf_.transform.translation.x = origin_.x;
  sensor_tf_.transform.translation.y = origin_.y;
  sensor_tf_.transform.translation.z = 0.0;
  sensor_tf_.transform.rotation.x = 0.0;
  sensor_tf_.transform.rotation.y = 0.0;
  sensor_tf_.transform.rotation.z = sin(0.5 * origin_.theta);
  sensor_tf_.transform.rotation.w = cos(0.5 * origin_.theta);
}

float RadiationSensor::CalcSource(b2Vec2 sensor_pos, flatland_msgs::RadSource source) {
  if (source.geometry == "point") {
    float dist = Distance(sensor_pos.x, sensor_pos.y, source.pose.position.x, source.pose.position.y);
    float value = source.value/pow(dist, 2);
    return value;
  } else if (source.geometry == "line") {
    float i_x, i_y;
    float h = DistancePointToLine(sensor_pos.x, sensor_pos.y,
                                  source.p1.x, source.p1.y,
                                  source.p2.x, source.p2.y,
                                  &i_x, &i_y);
    float L_L = Distance(i_x, i_y, source.p1.x, source.p1.y);
    float L_R = Distance(i_x, i_y, source.p2.x, source.p2.y);
    float value = source.value*(atan(L_L/h) + atan(L_R/h))/h;
    return value;
  } else {
    ROS_ERROR_NAMED("Radiation sensor", "Source geometry not specified. Must be 'point' or 'line'");
    return 0.0;
  }
}

float RadiationSensor::Distance(float x1, float y1, float x2, float y2) {
  float dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  return dist;
}

float RadiationSensor::DistancePointToLine(float x0, float y0, float x1, float y1, float x2, float y2, float* i_x, float* i_y) {
  float dist = abs((x2 - x1)*(y1 - y0) - (x1 - x0)*(y2 - y1));
  dist = dist/Distance(x1, y1, x2, y2);

  // find the point on the line closest to the point (x0, y0)
  float a = y1 - y2;
  float b = x2 - x1;
  float c = x1*y2 - x2*y1;
  *i_x = (b*(b*x0 - a*y0) - a*c)/(pow(a, 2) + pow(b, 2));
  *i_y = (a*(-b*x0 + a*y0) - b*c)/(pow(a, 2) + pow(b, 2));
  return dist;
}
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::RadiationSensor, flatland_server::ModelPlugin)
