// Copyright (C) 2020 by Krishneel Chaudhary
// DeepX-Inc, Tokyo

#include <node_registry/rosnode.hpp>

#include <string>
#include <algorithm>
#include <vector>

namespace node_registry {
namespace xnode {
XNode::XNode(
    const std::string node_name,
    const rclcpp::NodeOptions &option) :
    Node(node_name, rclcpp::NodeOptions(option)
         .automatically_declare_parameters_from_overrides(true)
         .allow_undeclared_parameters(true)),
    node_name_(node_name) {
  std::string line = std::string(80, '-');
  RCLCPP_INFO(this->get_logger(),
              "\n\033[32m%s\nNode started >>> %s/%s\n%s\033[0m",
              line.c_str(), this->get_namespace(), this->get_name(),
              line.c_str());

  auto use_sim_time = this->useSimTime();
  auto parameter = this->rclParam<bool>("use_sim_time", use_sim_time);
  auto result = this->set_parameter(parameter);

  if (use_sim_time) {
    RCLCPP_WARN(this->get_logger(),
                "Node %s will use SIM_TIME", node_name.c_str());
  }
}

XNode::~XNode() {
  this->shutdown();
}

void XNode::poke() {
  this->onInit();
  this->subscribe();
}

bool XNode::useSimTime() {
  try {
    auto data = std::string(std::getenv(SIM_VAR_NAME));
    std::transform(data.begin(), data.end(), data.begin(),
                   [](unsigned char c) {
                     return std::tolower(c);
                   });
    return data.compare("true") == 0 || data.compare("1") == 0;
  } catch (const std::logic_error &e) {
    return false;
  }
}

void XNode::shutdown() {
  RCLCPP_INFO(this->get_logger(),
              "\033[34mNode shutdown <<< %s\033[0m", this->node_name_.c_str());
}

template<typename T>
T XNode::getParameterValue(const std::string name,
                  const T default_value) {
  T param_value;
  auto param = this->get_parameter(name);
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    param_value = this->declare_parameter<T>(name, default_value);
    RCLCPP_WARN(this->get_logger(),
                "Parameter %s not set, using default", name.c_str());
  } else {
    param_value = param.get_value<T>();
  }
  return param_value;
}
}  // namespace xnode
}  // namespace node_registry

template rclcpp::Parameter node_registry::xnode::XNode::rclParam<bool>(
    const std::string, const bool) const;

template std::string node_registry::xnode::XNode::getParameterValue(
    const std::string, const std::string);
template int node_registry::xnode::XNode::getParameterValue(
    const std::string, const int);
template double node_registry::xnode::XNode::getParameterValue(
    const std::string, const double);
template float node_registry::xnode::XNode::getParameterValue(
    const std::string, const float);
template bool node_registry::xnode::XNode::getParameterValue(
    const std::string, const bool);
template std::vector<uint8_t> node_registry::xnode::XNode::getParameterValue(
    const std::string, const std::vector<uint8_t>);
template std::vector<bool> node_registry::xnode::XNode::getParameterValue(
    const std::string, const std::vector<bool>);
template std::vector<int64_t> node_registry::xnode::XNode::getParameterValue(
    const std::string, const std::vector<int64_t>);
template std::vector<double> node_registry::xnode::XNode::getParameterValue(
    const std::string, const std::vector<double>);
template std::vector<
    std::string> node_registry::xnode::XNode::getParameterValue(
    const std::string, const std::vector<std::string>);
