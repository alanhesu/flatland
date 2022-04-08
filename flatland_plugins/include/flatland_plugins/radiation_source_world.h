#include <flatland_plugins/update_timer.h>
#include <flatland_msgs/RadSources.h>
#include <flatland_msgs/SpawnRadSource.h>
#include <flatland_msgs/RadSource.h>
#include <flatland_server/world_plugin.h>
#include <flatland_server/world_source_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <thirdparty/ThreadPool.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include <thread>

#ifndef FLATLAND_PLUGINS_RADIATIONSOURCEWORLD_H
#define FLATLAND_PLUGINS_RADIATIONSOURCEWORLD_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class implements the model plugin class and provides laser data
 * for the given configurations
 */
class RadiationSourceWorld : public WorldSourcePlugin {
 public:
  std::string topic_;     ///< topic name to publish the laser scan
  double update_rate_;    ///< the rate laser scan will be published
  flatland_msgs::RadSources source_msg_;

  //int source_max_ = 20;
  //int source_size_ = 0;
  std::vector<flatland_msgs::RadSource> sources_;

  std::default_random_engine rng_;              ///< random generator

  ros::Publisher source_publisher_;             ///< ros laser topic publisher

  ros::ServiceServer spawn_source_service_;   ///< service for spawning radiation sources

  flatland_plugins::UpdateTimer update_timer_;                  ///< for controlling update rate

  // create separate nodehandle and callback queue to handle spawn source service calls
  ros::NodeHandle nh_rad_;
  ros::CallbackQueue rad_callback_queue_;
  ros::AsyncSpinner spinner_ = ros::AsyncSpinner(1, &rad_callback_queue_);

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  void PopulateSourceMsg(const Timekeeper &timekeeper);

  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

  bool SpawnRadSource(flatland_msgs::SpawnRadSource::Request &request, flatland_msgs::SpawnRadSource::Response &response);

  void AddSources(std::vector<boost::shared_ptr<ModelPlugin>> model_plugins) override;

  void AddSourceToArray(flatland_msgs::RadSource source);

  /**
   * @brief helper function to extract the paramters from the YAML Node
   * @param[in] config Plugin YAML Node
   */
  void ParseParameters(const YAML::Node &config);
};

};

#endif
