#include <flatland_server/model_plugin.h>

#ifndef FLATLAND_PLUGINS_SIMPLE_H
#define FLATLAND_PLUGINS_SIMPLE_H

using namespace flatland_server;
namespace flatland_plugins {

class Simple : public ModelPlugin {
public:
void OnInitialize(const YAML::Node &config) override;
};
};

#endif
