// ROS 2 发布器实现；编译禁用时构造会明确报错，不静默忽略 --ros。
#include "ros_output.hpp"
#include <stdexcept>
#ifdef AIM_HAVE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif
namespace aim::app {
struct RosOutput::Impl {
public:
// 初始化本程序使用的 ROS 上下文、节点和深度为 1 的默认可靠发布队列。
  Impl() {
#ifdef AIM_HAVE_ROS2
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("demo_aim_cpp_observer");
    publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "/demo/aim/solution", rclcpp::QoS(1));
#else
    throw std::runtime_error("ROS 2 not compiled; configure -DAIM_WITH_ROS2=ON "
                             "after sourcing Humble");
#endif
  }
// 发布当前 JSON 后处理就绪回调；此处没有启动后台 executor 线程。
  void publish(const std::string &text) {
#ifdef AIM_HAVE_ROS2
    std_msgs::msg::String message;
    message.data = text;
    publisher_->publish(message);
    rclcpp::spin_some(node_);
#else
    (void)text;
#endif
  }
// 先释放发布器和节点，再关闭本应用的 ROS 上下文。
  ~Impl() {
#ifdef AIM_HAVE_ROS2
    publisher_.reset();
    node_.reset();
    if (rclcpp::ok())
      rclcpp::shutdown();
#endif
  }

private:
#ifdef AIM_HAVE_ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
#endif
};

// 对外接口转发给隐藏实现，unique_ptr 负责清理资源。
RosOutput::RosOutput() : impl_(std::make_unique<Impl>()) {}
RosOutput::~RosOutput() = default;
// 转发单条观测；消息失联仍须消费端自行超时处理。
void RosOutput::publish(const std::string &text) { impl_->publish(text); }
} // namespace aim::app
