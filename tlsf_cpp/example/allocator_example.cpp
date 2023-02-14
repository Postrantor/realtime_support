// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tlsf_cpp/tlsf.hpp"

/// 使用 TLSFAllocator 作为分配器的模板别名。
template <typename T> using TLSFAllocator = tlsf_heap_allocator<T>;

int main(int argc, char ** argv)
{
  /// 使用 rclcpp 命名空间中的 AllocatorMemoryStrategy 类型。
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  /// 使用 Alloc 类型来定义 TLSFAllocator<void>。
  using Alloc = TLSFAllocator<void>;
  /// 初始化 ROS2 节点。
  rclcpp::init(argc, argv);

  /// 定义一个共享指针类型的节点。
  rclcpp::Node::SharedPtr node;

  /// 定义键列表，用于检查命令行参数是否启用了进程内通信。
  std::list<std::string> keys = {"intra", "intraprocess", "intra-process", "intra_process"};
  /// 设置一个布尔值表示是否使用进程内通信，默认为 false。
  bool intra_process = false;

  /// 如果有提供命令行参数，则检查是否匹配启用进程内通信的键。
  if (argc > 1) {
    for (auto & key : keys) {
      if (std::string(argv[1]) == key) {
        intra_process = true;
        break;
      }
    }
  }

  /// 根据是否启用进程内通信创建节点。
  if (intra_process) {
    printf("Intra-process pipeline is ON.\n");

    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);

    node = rclcpp::Node::make_shared("allocator_example", options);
  } else {
    printf("Intra-process pipeline is OFF.\n");

    auto options = rclcpp::NodeOptions().use_intra_process_comms(false);

    node = rclcpp::Node::make_shared("allocator_example", options);
  }

  /// 定义一个计数器变量。
  uint32_t counter = 0;
  /// 定义一个回调函数，当收到消息时更新计数器。
  auto callback = [&counter](std_msgs::msg::UInt32::SharedPtr msg) -> void {
    (void)msg;
    ++counter;
  };

  /// 创建一个自定义分配器，并将分配器传递给发布者和订阅者。
  auto alloc = std::make_shared<Alloc>();
  rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options;
  publisher_options.allocator = alloc;
  auto publisher =
    node->create_publisher<std_msgs::msg::UInt32>("allocator_example", 10, publisher_options);

  rclcpp::SubscriptionOptionsWithAllocator<Alloc> subscription_options;
  subscription_options.allocator = alloc;
  auto msg_mem_strat = std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<std_msgs::msg::UInt32, Alloc>>(alloc);
  auto subscriber = node->create_subscription<std_msgs::msg::UInt32>(
    "allocator_example", 10, callback, subscription_options, msg_mem_strat);

  /// 创建一个 MemoryStrategy，用于处理 Executor 在执行过程中的分配，
  /// 并将 MemoryStrategy 注入到 Executor 中。
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<Alloc>>(alloc);

  rclcpp::ExecutorOptions options;
  options.memory_strategy = memory_strategy;
  rclcpp::executors::SingleThreadedExecutor executor(options);

  /// 将我们的节点添加到执行器中。
  executor.add_node(node);

  /// 定义消息分配器和删除器，并创建一个唯一指针类型的消息。
  using MessageAllocTraits = rclcpp::allocator::AllocRebind<std_msgs::msg::UInt32, Alloc>;
  using MessageAlloc = MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, std_msgs::msg::UInt32>;
  using MessageUniquePtr = std::unique_ptr<std_msgs::msg::UInt32, MessageDeleter>;
  MessageDeleter message_deleter;
  MessageAlloc message_alloc = *alloc;
  rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &message_alloc);

  rclcpp::sleep_for(std::chrono::milliseconds(1));

  uint32_t i = 0;
  while (rclcpp::ok() && i < 100) {
    /// 使用自定义分配器创建消息，这样当 Executor 在执行路径上释放消息时，
    /// 它将使用自定义 deallocate。
    auto ptr = MessageAllocTraits::allocate(message_alloc, 1);
    MessageAllocTraits::construct(message_alloc, ptr);
    MessageUniquePtr msg(ptr, message_deleter);
    msg->data = i;
    ++i;
    publisher->publish(std::move(msg));
    rclcpp::sleep_for(std::chrono::milliseconds(1));
    executor.spin_some();
  }

  return 0;
}
