// 可选 ROS 诊断发布接口；应用只传 JSON 字符串，不依赖 ROS 消息类型。
#pragma once
#include <memory>
#include <string>
namespace aim::app {
// ROS types stay in the implementation; application code sees only strings.
// RAII 封装节点及发布器；无 ROS 的构建仍能包含本头文件。
class RosOutput {
public:
  RosOutput();
  ~RosOutput();
  RosOutput(const RosOutput &) = delete;
  RosOutput &operator=(const RosOutput &) = delete;
// 向 /demo/aim/solution 发布 std_msgs/String；这是观测诊断，不是飞行命令。
  void publish(const std::string &text);

private:
// PIMPL 将 rclcpp 类型与条件编译留在 .cpp 内，减少应用层耦合。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace aim::app
