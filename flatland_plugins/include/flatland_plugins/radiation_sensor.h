#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <flatland_msgs/RadSources.h>
#include <flatland_msgs/RadSource.h>
#include <flatland_msgs/RadSensor.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

#ifndef FLATLAND_PLUGINS_RADIATIONSENSOR_H
#define FLATLAND_PLUGINS_RADIATIONSENSOR_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class simulates a RADIATIONSENSOR receiver in Flatland
 */
class RadiationSensor : public ModelPlugin {
 public:
  std::string topic_;     ///< topic name to publish the sensor value
  std::string source_topic_; ///< source topic to subscribe to
  std::string frame_id_;  ///< RADIATIONSENSOR frame ID
  Body *body_;            ///< body the simulated RADIATIONSENSOR antenna attaches to
  Pose origin_;           ///< RADIATIONSENSOR sensor frame w.r.t the body
                        /// altitude
  double update_rate_;  ///< RADIATIONSENSOR fix publish rate
  bool broadcast_tf_;   ///< whether to broadcast laser origin w.r.t body

  ros::Publisher sensor_publisher_;             ///< RADIATIONSENSOR fix topic publisher
  ros::Subscriber source_subscriber_;
  tf::TransformBroadcaster tf_broadcaster_;  ///< broadcast RADIATIONSENSOR frame
  geometry_msgs::TransformStamped sensor_tf_;   ///< tf from body to RADIATIONSENSOR frame
  UpdateTimer update_timer_;                 ///< for controlling update rate

  Eigen::Matrix3f m_body_to_sensor_;  ///< tf from body to GPS

  flatland_msgs::RadSources source_msg_;
  flatland_msgs::RadSensor sensor_msg_;

  void SourceCallback(const flatland_msgs::RadSources& msg);

  void UpdateSource(const Timekeeper &timekeeper);

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

  /**
   * @brief Helper function to extract the paramters from the YAML Node
   * @param[in] config Plugin YAML Node
   */
  void ParseParameters(const YAML::Node &config);

  float CalcSource(b2Vec2 sensor_pos, flatland_msgs::RadSource source);

  float Distance(float x1, float y1, float x2, float y2);

  float DistancePointToLine(float x0, float y0, float x1, float y1, float x2, float y2, float* i_x, float* i_y);
};
}

#endif  // FLATLAND_PLUGINS_RADIATIONSENSOR_H
