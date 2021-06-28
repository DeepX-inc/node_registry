// Copyright (c) 2021, DeepX-inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// @author Krishneel Chaudhary
//

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

/**
 * asdsd
 */
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
template std::vector<std::string> node_registry::xnode::XNode::getParameterValue(
    const std::string, const std::vector<std::string>);
