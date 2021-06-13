# node_registry

[wiki](https://github.com/DeepX-inc/node_registry/wiki)

---
---

<span style="color:MediumSeaGreen;font-size:30px">Python Talker & Listner Node</span>.

## Simple talker node using node registry

```python
from node_registry.decorators import register, rosnode
from std_msgs.msg import String
_pub_topic = 'topic'


@rosnode
def node() -> str:
    return 'Minimal_publisher'


rosnode.node.i = 0


@rosnode.publisher(String, _pub_topic)
@rosnode.timer(1.0)
def timer_cb():
    pub = rosnode.get_publisher(_pub_topic)
    msg = String()
    rosnode.node.i += 1
    msg.data = 'Hello World %d' % rosnode.node.i
    pub.publish(msg)
    rosnode.logger.info('Publishing: "%s"' % msg.data)


register()
```

---


## Simple listner node using node registry

```python
from node_registry.decorators import register, rosnode
from std_msgs.msg import String

_sub_topic = 'topic'


@rosnode
def node() -> str:
    return 'Minimal_subscriber'


@rosnode.subscribe(String, _sub_topic)
def func_for_subs(msg):
    rosnode.logger.info('I heard: "%s"' % msg.data)


register()
```
<br>
<br>
---
***

<span style="color:MediumSeaGreen;font-size:30px">Cpp Talker & Listner Node</span>.

---
## Simple talker node using node registry
```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <node_registry/rosnode.hpp>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using node_registry::xnode::XNode;

class MinimalPublisher : public node_registry::xnode::XNode
{
  public:
    MinimalPublisher()
    : XNode("Minimal_publisher",rclcpp::NodeOptions()),count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

    void onInit(){

    }

    void subscribe(){

    }

    void shutdown(){


    }
  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

---
## Simple listner node using node registry
```cpp
#include <memory>

#include <node_registry/rosnode.hpp>
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public node_registry::xnode::XNode
{
  public:
    MinimalSubscriber()
    : XNode("minimal_subscriber",rclcpp::NodeOptions())
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }


    void onInit(){

    }

    void subscribe(){

    }

    void shutdown(){


    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```