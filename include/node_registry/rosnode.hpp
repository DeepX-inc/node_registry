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
