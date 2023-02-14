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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "std_msgs/msg/u_int32.hpp"
#include "tlsf_cpp/tlsf.hpp"

#ifdef RMW_IMPLEMENTATION
#define CLASSNAME_(NAME, SUFFIX) NAME##__##SUFFIX
#define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
#define CLASSNAME(NAME, SUFFIX) NAME
#endif

template <typename T = void> using TLSFAllocator = tlsf_heap_allocator<T>;

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

/**
 * @class CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION)
 * @brief 一个基于 rclcpp 的测试类，用于测试自定义分配器。
 *        A test class based on rclcpp for testing custom allocator.
 */
class CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION) : public ::testing::Test
{
protected:
  // 测试名称 (Test name)
  std::string test_name_;

  // 节点共享指针 (Node shared pointer)
  rclcpp::Node::SharedPtr node_;
  // 单线程执行器共享指针 (Single-threaded executor shared pointer)
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  // 内存策略共享指针 (Memory strategy shared pointer)
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;
  // 发布者共享指针 (Publisher shared pointer)
  rclcpp::Publisher<std_msgs::msg::UInt32, TLSFAllocator<void>>::SharedPtr publisher_;
  // 消息内存策略共享指针 (Message memory strategy shared pointer)
  rclcpp::message_memory_strategy::MessageMemoryStrategy<
    std_msgs::msg::UInt32,
    TLSFAllocator<void>>::SharedPtr msg_memory_strategy_;
  // 分配器共享指针 (Allocator shared pointer)
  std::shared_ptr<TLSFAllocator<void>> alloc;
  // 订阅选项 (Subscription options)
  rclcpp::SubscriptionOptionsWithAllocator<TLSFAllocator<void>> subscription_options_;
  // 发布选项 (Publisher options)
  rclcpp::PublisherOptionsWithAllocator<TLSFAllocator<void>> publisher_options_;

  // 是否使用内部进程通信 (Whether to use intra-process communication)
  bool intra_process_;

  // UInt32 分配器 (UInt32 allocator)
  using UInt32Allocator = TLSFAllocator<std_msgs::msg::UInt32>;
  // UInt32 删除器 (UInt32 deleter)
  using UInt32Deleter = rclcpp::allocator::Deleter<UInt32Allocator, std_msgs::msg::UInt32>;

  /**
   * @brief 初始化测试类
   *        Initialize the test class.
   *
   * @param[in] intra_process 是否使用内部进程通信
   *                          Whether to use intra-process communication.
   * @param[in] name 测试名称 (Test name)
   */
  void initialize(bool intra_process, const std::string & name)
  {
    test_name_ = name;
    intra_process_ = intra_process;

    auto context = rclcpp::contexts::get_global_default_context();
    auto options =
      rclcpp::NodeOptions().context(context).use_global_arguments(true).use_intra_process_comms(
        intra_process);

    node_ = rclcpp::Node::make_shared(name, options);
    alloc = std::make_shared<TLSFAllocator<void>>();
    subscription_options_.allocator = alloc;
    publisher_options_.allocator = alloc;
    msg_memory_strategy_ = std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy<
      std_msgs::msg::UInt32, TLSFAllocator<void>>>(alloc);
    publisher_ = node_->create_publisher<std_msgs::msg::UInt32>(name, 10, publisher_options_);
    memory_strategy_ = std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>(alloc);

    rclcpp::ExecutorOptions executor_options;
    executor_options.memory_strategy = memory_strategy_;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);

    executor_->add_node(node_);
  }

  /**
   * @brief 默认构造函数 (Default constructor)
   */
  CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION)() {}

  /**
   * @brief 析构函数 (Destructor)
   */
  ~CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION)() {}
};

/*!
 * \brief 测试分配器类型特性 (Test allocator type traits)
 *
 * \tparam AllocatorTest 分配器测试类 (Allocator test class)
 * \tparam RMW_IMPLEMENTATION ROS中间件实现 (ROS middleware implementation)
 */
TEST_F(CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION), type_traits_test)
{
  // 使用TLSF分配器为std_msgs::msg::UInt32消息类型分配内存 (Use TLSF allocator to allocate memory for std_msgs::msg::UInt32 message type)
  using UInt32TLSFAllocator = TLSFAllocator<std_msgs::msg::UInt32>;
  // 使用rclcpp::allocator::Deleter来删除使用TLSF分配器分配的std_msgs::msg::UInt32对象 (Use rclcpp::allocator::Deleter to delete std_msgs::msg::UInt32 objects allocated with TLSF allocator)
  using UInt32TLSFDeleter = rclcpp::allocator::Deleter<UInt32TLSFAllocator, std_msgs::msg::UInt32>;

  // 定义一个回调函数，接受一个带有TLSF分配器的std_msgs::msg::UInt32唯一指针 (Define a callback function that accepts a unique pointer of std_msgs::msg::UInt32 with TLSF allocator)
  auto cb_tlsf = [](std_msgs::msg::UInt32::UniquePtrWithDeleter<UInt32TLSFDeleter> msg) -> void {
    // 不对msg进行任何操作 (No operation on msg)
    (void)msg;
  };
  // 静态断言检查cb_tlsf回调函数是否具有正确的消息类型 (Static assert to check if cb_tlsf callback has the correct message type)
  static_assert(
    std::is_same<
      std_msgs::msg::UInt32,
      rclcpp::subscription_traits::has_message_type<decltype(cb_tlsf)>::type>::value,
    "tlsf unique ptr failed");

  // 使用std::allocator为std_msgs::msg::UInt32消息类型分配内存 (Use std::allocator to allocate memory for std_msgs::msg::UInt32 message type)
  using UInt32VoidAllocator = std::allocator<std_msgs::msg::UInt32>;
  // 使用rclcpp::allocator::Deleter来删除使用std::allocator分配的std_msgs::msg::UInt32对象 (Use rclcpp::allocator::Deleter to delete std_msgs::msg::UInt32 objects allocated with std::allocator)
  using UInt32VoidDeleter = rclcpp::allocator::Deleter<UInt32VoidAllocator, std_msgs::msg::UInt32>;

  // 定义一个回调函数，接受一个带有std::allocator的std_msgs::msg::UInt32唯一指针 (Define a callback function that accepts a unique pointer of std_msgs::msg::UInt32 with std::allocator)
  auto cb_void = [](std_msgs::msg::UInt32::UniquePtrWithDeleter<UInt32VoidDeleter> msg) -> void {
    // 不对msg进行任何操作 (No operation on msg)
    (void)msg;
  };
  // 静态断言检查cb_void回调函数是否具有正确的消息类型 (Static assert to check if cb_void callback has the correct message type)
  static_assert(
    std::is_same<
      std_msgs::msg::UInt32,
      rclcpp::subscription_traits::has_message_type<decltype(cb_void)>::type>::value,
    "void unique ptr failed");
}

/**
// TODO(wjwwood): re-enable this test when the allocator has been added back to the
//   intra-process manager.
//   See: https://github.com/ros2/realtime_support/pull/80#issuecomment-545419570
TEST_F(CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION), allocator_unique_ptr) {
  initialize(true, "allocator_unique_ptr");
  size_t counter = 0;
  auto callback =
    [&counter](std::unique_ptr<std_msgs::msg::UInt32, UInt32Deleter> msg) -> void
    {
      EXPECT_EQ(counter, msg->data);
      counter++;
    };

  static_assert(
    std::is_same<
      std_msgs::msg::UInt32,
      rclcpp::subscription_traits::has_message_type<decltype(callback)>::type>::value,
    "passing a std::unique_ptr of test_msgs::msg::Empty has message type Empty");

  auto subscriber = node_->create_subscription<std_msgs::msg::UInt32>(
    "allocator_unique_ptr", 10, callback, subscription_options_, msg_memory_strategy_);

  TLSFAllocator<std_msgs::msg::UInt32> msg_alloc;

  // After test_initialization, global new should only be called from within TLSFAllocator.
  test_init = true;
  for (uint32_t i = 0; i < iterations; i++) {
    auto msg = std::unique_ptr<std_msgs::msg::UInt32, UInt32Deleter>(
      std::allocator_traits<UInt32Allocator>::allocate(msg_alloc, 1));
    msg->data = i;
    publisher_->publish(std::move(msg));
    rclcpp::sleep_for(std::chrono::milliseconds(1));
    executor_->spin_some();
  }
  test_init = false;
  EXPECT_FALSE(fail);
  fail = false;
}
*/

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
