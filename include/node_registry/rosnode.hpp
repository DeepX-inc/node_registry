// Copyright (C) 2020 by Krishneel Chaudhary
// DeepX-Inc, Tokyo

#ifndef NODE_REGISTRY__ROSNODE_HPP_
#define NODE_REGISTRY__ROSNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/parameter.hpp>

#include <string>

#define SIM_VAR_NAME "USE_SIM_TIME"

namespace node_registry {
namespace xnode {

class XNode: public rclcpp::Node {
 private:
  std::string node_name_;
  bool useSimTime();

 protected:
  virtual void onInit() = 0;
  virtual void subscribe() = 0;
  virtual void shutdown();

  void poke();

 public:
  XNode(const std::string, const rclcpp::NodeOptions &);
  ~XNode();

  template<typename T>
  rclcpp::Parameter rclParam(const std::string name,
                             const T value) const {
    return rclcpp::Parameter(name, value);
  }

  template<typename T>
  T getParameterValue(const std::string name, const T default_value);
};
}  // namespace xnode
}  // namespace node_registry

#endif  // NODE_REGISTRY__ROSNODE_HPP_
