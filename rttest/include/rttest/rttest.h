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

#ifndef RTTEST__RTTEST_H_
#define RTTEST__RTTEST_H_

#include <sched.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief rttest_params 结构体，用于存储实时测试参数 (Structure for storing real-time test parameters)
 */
struct rttest_params
{
  size_t iterations;              ///< 测试迭代次数 (Number of test iterations)
  struct timespec update_period;  ///< 更新周期 (Update period)
  size_t sched_policy;            ///< 调度策略 (Scheduling policy)
  int sched_priority;             ///< 调度优先级 (Scheduling priority)
  size_t stack_size;              ///< 栈大小 (Stack size)
  uint64_t prefault_dynamic_size; ///< 预处理动态内存大小 (Prefault dynamic memory size)

  // TODO(dirk-thomas) 当前这个指针从未被释放或复制 (This pointer is currently never deallocated or copied)
  // 所以无论什么值被赋值，都必须永远有效 (So whatever value is being assigned must stay valid forever)
  char * filename; ///< 文件名 (Filename)
};

/**
 * @brief rttest_results 结构体，用于存储实时测试结果 (Structure for storing real-time test results)
 */
struct rttest_results
{
  size_t iteration; ///< 本结果描述的最大迭代次数 (Max iteration that this result describes)
  int64_t min_latency;   ///< 最小延迟 (Minimum latency)
  int64_t max_latency;   ///< 最大延迟 (Maximum latency)
  double mean_latency;   ///< 平均延迟 (Mean latency)
  double latency_stddev; ///< 延迟标准差 (Latency standard deviation)

  size_t minor_pagefaults; ///< 次要缺页故障数量 (Number of minor page faults)
  size_t major_pagefaults; ///< 主要缺页故障数量 (Number of major page faults)
};

/// \brief 初始化 rttest 并使用参数
/// Initialize rttest with arguments
/// \param[in] argc 参数向量的大小
/// Size of argument vector
/// \param[out] argv 参数向量
/// Argument vector
/// \return 传播到 main 的错误代码
/// Error code to propagate to main
int rttest_read_args(int argc, char ** argv);

/// \brief 初始化 rttest。预分配样本缓冲区，存储用户参数，必要时锁定内存
/// Initialize rttest. Preallocate the sample buffer, store user parameters, lock memory if necessary
/// 非实时安全。
/// Not real time safe.
/// \param[in] iterations 要旋转的迭代次数
/// How many iterations to spin for
/// \param[in] update_period 表示旋转周期的时间间隔
/// Time interval representing the spin period
/// \param[in] sched_policy 调度策略，例如循环调度，先进先出
/// Scheduling policy, e.g. round robin, FIFO
/// \param[in] sched_priority 线程优先级
/// The thread priority
/// \param[in] stack_size 当调用 rttest_prefault_stack() 时要预设的字节数
/// How many bytes to prefault when rttest_prefault_stack() is called.
/// \param[in] prefault_dynamic_size 在堆上预设的字节数
/// How many bytes to prefault on the heap.
/// \param[in] filename 保存结果的文件名
/// Name of the file to save results to.
/// \return 传播到 main 的错误代码
/// Error code to propagate to main
int rttest_init(
  size_t iterations,
  struct timespec update_period,
  size_t sched_policy,
  int sched_priority,
  size_t stack_size,
  uint64_t prefault_dynamic_size,
  char * filename);

/// \brief 使用当前 rttest 参数填充 rttest_params 结构。
/// Fill an rttest_params struct with the current rttest params.
/// \param[in] params 要填充的结构的引用
/// Reference to the struct to fill in
/// \return 错误代码
/// Error code
int rttest_get_params(struct rttest_params * params);

/// \brief 为新线程创建一个新的 rttest 实例。
/// Create a new rttest instance for a new thread.
/// 线程的参数基于第一个调用 rttest_init 的线程。
/// The thread's parameters are based on the first thread that called rttest_init.
/// 用户创建线程后直接调用。
/// To be called directly after the user creates the thread.
/// \return 传播到 main 的错误代码
/// Error code to propagate to main
int rttest_init_new_thread();

/// \brief 在指定的唤醒周期内旋转指定的迭代次数。
/// Spin at the specified wakeup period for the specified number of iterations.
/// \param[in] user_function 唤醒时执行的函数指针
/// Function pointer to execute on wakeup
/// \param[in] args 函数的参数
/// Arguments to the function
/// \return 传播到 main 的错误代码
/// Error code to propagate to main
int rttest_spin(void * (*user_function)(void *), void * args);

// TODO(jacquelinekay) 更好的用户函数签名
/// \brief 在指定的唤醒周期内旋转指定的迭代次数。rttest_spin 将尝试根据 update_period 对 user_function 的执行进行计时。
/// 在所有内容初始化之后调用此方法。
/// \param[in] user_function 指向要在唤醒时执行的函数的函数指针。
/// \param[in] args 函数的参数
/// \param[in] update_period 更新周期（覆盖 rttest_init 中读取的参数）
/// \param[in] iterations 迭代次数（覆盖 rttest_init 中读取的参数）
/// \return 传播给主函数的错误代码
/// \brief Spin at the specified wakeup period for the specified number of
/// iterations. rttest_spin will attempt to time the execution of user_function
/// according to update_period.
/// Call this after everything has been initialized.
/// \param[in] user_function Function pointer to execute on wakeup.
/// \param[in] args Arguments to the function
/// \param[in] update_period Update period (overrides param read in rttest_init)
/// \param[in] iterations Iterations (overrides param read in rttest_init)
/// \return Error code to propagate to main
int rttest_spin_period(
  void * (*user_function)(void *),
  void * args,
  const struct timespec * update_period,
  const size_t iterations);

/// \brief 根据开始时间、更新周期和 spin 调用的迭代来安排一个函数调用。
/// 唤醒的统计信息将作为数据缓冲区中的第 'i' 个条目进行收集。
/// \param[in] user_function 指向要在中断时执行的函数的函数指针。
/// \param[in] update_period
/// \param[out] 传播给主函数的错误代码。
/// \return 传播给主函数的错误代码
/// \brief Schedule a function call based on the start time, update period,
/// and the iteration of the spin call.
/// The statistics of the wakeup will be collected as the 'ith' entry in the data buffer.
/// \param[in] user_function Function pointer to execute on interrupt.
/// \param[out] Error code to propagate to main function.
/// \return Error code to propagate to main
int rttest_spin_once_period(
  void * (*user_function)(void *),
  void * args,
  const struct timespec * start_time,
  const struct timespec * update_period,
  const size_t i);

/// \brief 根据开始时间、更新周期和 spin 调用的迭代来安排一个函数调用。
/// 唤醒的统计信息将作为数据缓冲区中的第 'i' 个条目进行收集。
/// TODO: 实现异步调度/记录
/// \param[in] user_function 指向要在中断时执行的函数的函数指针。
/// \param[out] 传播给主函数的错误代码。
/// \return 传播给主函数的错误代码
/// \brief Schedule a function call based on the start time, update period,
/// and the iteration of the spin call.
/// The statistics of the wakeup will be collected as the 'ith' entry in the data buffer.
/// TODO: implement asynchronous scheduling/logging
/// \param[in] user_function Function pointer to execute on interrupt.
/// \param[out] Error code to propagate to main function.
/// \return Error code to propagate to main
int rttest_spin_once(
  void * (*user_function)(void *), void * args, const struct timespec * start_time, const size_t i);

/// \brief 使用 mlockall 锁定当前分页内存。
/// \return 传播给主函数的错误代码
/// \brief Lock currently paged memory using mlockall.
/// \return Error code to propagate to main
int rttest_lock_memory();

/// \brief 预设堆栈大小。
/// \param[in] stack_size 堆栈的大小
/// \return 传播给主函数的错误代码
/// \brief Prefault the stack.
/// \param[in] stack_size The size of the stack
/// \return Error code to propagate to main
int rttest_prefault_stack_size(const size_t stack_size);

/// \brief 使用默认堆栈大小预设堆栈。
/// \return 传播给主函数的错误代码
/// \brief Prefault the stack using default stack size.
/// \return Error code to propagate to main
int rttest_prefault_stack();

/// \brief 提交基于此过程已缓存的内存的动态内存池，通过检查页面错误的数量。
/// (Commit a pool of dynamic memory based on the memory already cached
/// by this process by checking the number of pagefaults.)
/// \return 传播到主程序的错误代码 (Error code to propagate to main)
int rttest_lock_and_prefault_dynamic();

/// \brief 为此线程（pthreads）设置优先级和调度策略
/// (Set the priority and scheduling policy for this thread (pthreads))
/// \param[in] sched_priority 调度优先级。最大值为99。 (The scheduling priority. Max is 99.)
/// \param[in] policy 调度策略（FIFO、循环等）(The scheduling policy (FIFO, Round Robin, etc.))
/// \return 传播到主程序的错误代码 (Error code to propagate to main)
int rttest_set_sched_priority(const size_t sched_priority, const int policy);

/// \brief 使用默认参数为此线程设置优先级和调度策略
/// (Set the priority and scheduling policy for this thread using
/// default parameters.)
/// \return 传播到主程序的错误代码 (Error code to propagate to main)
int rttest_set_thread_default_priority();

/// \brief 在特定迭代中获取rusage（页面错误）并在样本缓冲区中记录
/// (Get rusage (pagefaults) and record in the sample buffer at a
/// particular iteration)
/// \param[in] i 存储页面错误信息的索引。(Index at which to store the pagefault information.)
/// \return 传播到主程序的错误代码 (Error code to propagate to main)
int rttest_get_next_rusage(size_t i);

/// \brief 计算统计数据并填充给定的结果结构。
/// (Calculate statistics and fill the given results struct.)
/// \param[in] results 用统计数据填充的结果结构。(The results struct to fill with statistics.)
/// \return 如果结果结构为NULL或计算无效，则返回错误代码 (Error code if results struct is NULL or if calculations invalid)
int rttest_calculate_statistics(struct rttest_results * results);

/// \brief 获取累积统计信息
/// (Get accumulated statistics)
/// \return 如果结果结构为NULL，则返回错误代码 (Error code if results struct is NULL)
int rttest_get_statistics(struct rttest_results * results);

/// \brief 获取给定迭代处的延迟样本。
/// (Get latency sample at the given iteration.)
/// \param[in] iteration 从测试中获取样本的迭代 (Iteration of the test to get the sample from)
/// \param[out] 结果样本：预期唤醒时间与实际唤醒时间之间的纳秒数 (The resulting sample: time in nanoseconds between the expected
/// wakeup time and the actual wakeup time)
int rttest_get_sample_at(const size_t iteration, int64_t * sample);

/// \brief 将样本缓冲区写入文件。 (Write the sample buffer to a file.)
/// \return 传播到 main 的错误代码 (Error code to propagate to main)
int rttest_write_results();

/// \brief 将样本缓冲区写入文件。 (Write the sample buffer to a file.)
/// \param[in] 存储样本缓冲区的文件名；覆盖默认参数。 (Filename to store the sample buffer; overrides default param.)
/// \return 传播到 main 的错误代码 (Error code to propagate to main)
int rttest_write_results_file(char * filename);

/// \brief 释放内存并清理 (Free memory and cleanup)
/// \return 传播到 main 的错误代码 (Error code to propagate to main)
int rttest_finish();

/// \brief 检查 rttest 实例是否正在运行（准备收集数据或正在收集数据）(Check if the rttest instance is running (ready to collect data or collecting data))
/// \return 如果实例未运行，则返回0；如果正在运行，则返回1。 (0 if the instance is not running, 1 if it is running.)
int rttest_running();

#ifdef __cplusplus
}
#endif

#endif // RTTEST__RTTEST_H_
