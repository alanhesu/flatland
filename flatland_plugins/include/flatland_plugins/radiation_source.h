#include <flatland_plugins/update_timer.h>
#include <flatland_msgs/SpawnRadSource.h>
#include <flatland_msgs/RadSource.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <ros/ros.h>

#ifndef FLATLAND_PLUGINS_RADIATIONSOURCE_H
#define FLATLAND_PLUGINS_RADIATIONSOURCE_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class simulates a GPS receiver in Flatland
 */
class RadiationSource : public ModelPlugin {
 public:
  Body *body_;            ///< body the simulated GPS antenna attaches to
  Pose origin_;           ///< GPS sensor frame w.r.t the body
  double update_rate_;  ///< GPS fix publish rate
  std::string geometry_;
  float value_;
  std::string name_;
  bool spawned_ = false;          /// keep track of if the spawn service has been called once
  std::vector<b2Vec2> points_;    ///< 2 points for line source

  UpdateTimer update_timer_;                 ///< for controlling update rate

  ros::ServiceClient source_client_;
  flatland_msgs::SpawnRadSource spawn_rad_srv_;

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

  void GetSource(flatland_msgs::RadSource* source);

  void SpawnSource();

  /**
   * @brief Helper function to extract the paramters from the YAML Node
   * @param[in] config Plugin YAML Node
   */
  void ParseParameters(const YAML::Node &config);
};
}

#endif  // FLATLAND_PLUGINS_RADIATIONSOURCE_H
