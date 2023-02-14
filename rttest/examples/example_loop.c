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
#include <sched.h>
#include <stdio.h>

#include <rttest/rttest.h>

/**
 * @file main.c
 * @brief ROS2 RCLCPP 示例代码 (ROS2 RCLCPP example code)
 */

int i = 0; ///< 计数器，用于记录循环回调函数的执行次数 (Counter for counting the number of times the loop callback function is executed)

/**
 * @brief 循环回调函数 (Loop callback function)
 * @param args 传递给回调函数的参数 (Arguments passed to the callback function)
 * @return 返回值始终为0 (Return value is always 0)
 */
void * my_loop_callback(void * args)
{
  ++i; // 增加计数器的值 (Increment the counter value)
  return 0;
}

/**
 * @brief 主函数 (Main function)
 * @param argc 参数个数 (Number of arguments)
 * @param argv 参数值数组 (Array of argument values)
 * @return 程序执行结果 (Program execution result)
 */
int main(int argc, char ** argv)
{
  // 设置调度优先级为98，使用SCHED_RR调度策略 (Set scheduling priority to 98 and use SCHED_RR scheduling policy)
  rttest_set_sched_priority(98, SCHED_RR);

  // 读取命令行参数 (Read command line arguments)
  if (rttest_read_args(argc, argv) != 0) {
    perror("Couldn't read arguments for rttest");
    return -1;
  }

  // 锁定内存 (Lock memory)
  if (rttest_lock_memory() != 0) {
    perror("Couldn't lock memory");
    return -1;
  }

  // 锁定并预分配动态内存 (Lock and pre-allocate dynamic memory)
  rttest_lock_and_prefault_dynamic();

  // 执行循环回调函数 (Execute loop callback function)
  rttest_spin(my_loop_callback, NULL);

  // 将测试结果写入文件 (Write test results to a file)
  rttest_write_results();

  // 结束实时测试 (Finish real-time testing)
  rttest_finish();

  return 0;
}
