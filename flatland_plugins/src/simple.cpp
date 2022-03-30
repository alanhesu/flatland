#include <flatland_plugins/simple.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

using namespace flatland_server;
namespace flatland_plugins {

void Simple::OnInitialize(const YAML::Node &config) {
ROS_ERROR_STREAM("Hello world");
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Simple, flatland_server::ModelPlugin)
