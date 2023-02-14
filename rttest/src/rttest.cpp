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

#include "rttest/rttest.h"

#include <alloca.h>
#include <limits.h>
#include <malloc.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <ios>
#include <map>
#include <numeric>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rttest/math_utils.hpp"
#include "rttest/utils.hpp"

/**
 * @class rttest_sample_buffer
 * @brief 一个用于存储实时测试数据的缓冲区类 (A buffer class for storing real-time test data)
 */
class rttest_sample_buffer
{
public:
  /**
   * @brief 默认构造函数 (Default constructor)
   */
  rttest_sample_buffer() = default;

  /**
   * @brief 默认析构函数 (Default destructor)
   */
  ~rttest_sample_buffer() = default;

  /**
   * @brief 拷贝构造函数 (Copy constructor)
   * @param other 另一个 rttest_sample_buffer 对象 (Another rttest_sample_buffer object)
   */
  rttest_sample_buffer(const rttest_sample_buffer & other) = default;

  /**
   * @brief 赋值运算符重载 (Assignment operator overload)
   * @param other 另一个 rttest_sample_buffer 对象 (Another rttest_sample_buffer object)
   * @return 当前对象的引用 (Reference to the current object)
   */
  rttest_sample_buffer & operator=(const rttest_sample_buffer & other) = default;

  /**
   * @brief 调整缓冲区大小 (Resize the buffer)
   * @param new_buffer_size 新的缓冲区大小 (New buffer size)
   */
  void resize(size_t new_buffer_size)
  {
    this->latency_samples.resize(
      new_buffer_size); // 调整延迟样本容器大小 (Resize latency samples container)
    this->major_pagefaults.resize(
      new_buffer_size); // 调整主要页面错误容器大小 (Resize major pagefaults container)
    this->minor_pagefaults.resize(
      new_buffer_size); // 调整次要页面错误容器大小 (Resize minor pagefaults container)
  }

  // 存储以纳秒为单位的延迟 (Stored in nanoseconds)
  // 负延迟表示事件提前发生（不太可能）(A negative latency means that the event was early (unlikely))
  std::vector<int64_t> latency_samples;

  // 主要页面错误数量 (Major pagefaults count)
  std::vector<size_t> major_pagefaults;

  // 次要页面错误数量 (Minor pagefaults count)
  std::vector<size_t> minor_pagefaults;
};

/**
 * @class Rttest
 * @brief 实时性测试类，用于评估执行时间和抖动的实时性。 (Real-time test class for evaluating execution time and jitter.)
 */
class Rttest
{
private:
  struct rttest_params params; ///< 测试参数结构体 (Test parameters structure)
  rttest_sample_buffer
    sample_buffer; ///< 样本缓冲区，用于存储每次迭代的结果 (Sample buffer for storing results of each iteration)
  struct rusage prev_usage; ///< 上一次资源使用情况 (Previous resource usage)

  pthread_t thread_id; ///< 线程ID (Thread ID)

  /**
   * @brief 记录抖动值 (Record jitter values)
   * @param deadline 期望完成时间 (Expected completion time)
   * @param result_time 实际完成时间 (Actual completion time)
   * @param iteration 迭代次数 (Iteration number)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int record_jitter(
    const struct timespec * deadline, const struct timespec * result_time, const size_t iteration);

  /**
   * @brief 累计统计数据 (Accumulate statistical data)
   * @param iteration 迭代次数 (Iteration number)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int accumulate_statistics(size_t iteration);

public:
  int running = 0;                  ///< 运行状态标志 (Running status flag)
  struct rttest_results results;    ///< 测试结果结构体 (Test results structure)
  bool results_initialized = false; ///< 结果是否初始化标志 (Results initialized flag)

  Rttest();  ///< 构造函数 (Constructor)
  ~Rttest(); ///< 析构函数 (Destructor)

  /**
   * @brief 从命令行参数中读取测试设置 (Read test settings from command line arguments)
   * @param argc 参数数量 (Number of arguments)
   * @param argv 参数值数组 (Array of argument values)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int read_args(int argc, char ** argv);

  /**
   * @brief 初始化实时性测试 (Initialize real-time test)
   * @param iterations 迭代次数 (Number of iterations)
   * @param update_period 更新周期 (Update period)
   * @param sched_policy 调度策略 (Scheduling policy)
   * @param sched_priority 调度优先级 (Scheduling priority)
   * @param stack_size 栈大小 (Stack size)
   * @param prefault_dynamic_size 预锁定动态内存大小 (Prefault dynamic memory size)
   * @param filename 输出结果文件名 (Output results file name)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int init(
    size_t iterations,
    struct timespec update_period,
    size_t sched_policy,
    int sched_priority,
    size_t stack_size,
    uint64_t prefault_dynamic_size,
    char * filename);

  /**
   * @brief 启动实时性测试 (Start real-time test)
   * @param user_function 用户自定义函数指针 (User-defined function pointer)
   * @param args 用户自定义函数参数 (User-defined function arguments)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int spin(void * (*user_function)(void *), void * args);

  /**
   * @brief 启动周期性的实时性测试 (Start periodic real-time test)
   * @param user_function 用户自定义函数指针 (User-defined function pointer)
   * @param args 用户自定义函数参数 (User-defined function arguments)
   * @param update_period 更新周期 (Update period)
   * @param iterations 迭代次数 (Number of iterations)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int spin_period(
    void * (*user_function)(void *),
    void * args,
    const struct timespec * update_period,
    const size_t iterations);

  /**
   * @brief 执行一次实时性测试 (Perform a single real-time test)
   * @param user_function 用户自定义函数指针 (User-defined function pointer)
   * @param args 用户自定义函数参数 (User-defined function arguments)
   * @param start_time 开始时间 (Start time)
   * @param i 迭代次数 (Iteration number)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int spin_once(
    void * (*user_function)(void *),
    void * args,
    const struct timespec * start_time,
    const size_t i);

  /**
   * @brief 执行一次周期性的实时性测试 (Perform a single periodic real-time test)
   * @param user_function 用户自定义函数指针 (User-defined function pointer)
   * @param args 用户自定义函数参数 (User-defined function arguments)
   * @param start_time 开始时间 (Start time)
   * @param update_period 更新周期 (Update period)
   * @param i 迭代次数 (Iteration number)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int spin_once(
    void * (*user_function)(void *),
    void * args,
    const struct timespec * start_time,
    const struct timespec * update_period,
    const size_t i);

  /**
   * @brief 锁定内存 (Lock memory)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int lock_memory();

  /**
   * @brief 锁定并预先分配动态内存 (Lock and prefault dynamic memory)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int lock_and_prefault_dynamic();

  /**
   * @brief 预先分配栈内存 (Prefault stack memory)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int prefault_stack();

  /**
   * @brief 设置线程默认优先级 (Set thread default priority)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int set_thread_default_priority();

  /**
   * @brief 获取下一次资源使用情况 (Get next resource usage)
   * @param i 迭代次数 (Iteration number)
   * @return 成功返回0，失败返回错误码 (Return 0 if successful, error code if failed)
   */
  int get_next_rusage(size_t i);

  /**
 * @brief 计算统计数据 (Calculate statistics)
 * @param results 结果结构体指针 (Pointer to the results structure)
 * @return 成功返回0，失败返回错误代码 (Returns 0 on success, error code on failure)
 */
  int calculate_statistics(struct rttest_results * results);

  /**
 * @brief 获取指定迭代次数的样本值 (Get the sample value at the specified iteration)
 * @param iteration 迭代次数 (Iteration count)
 * @param sample 输出参数，用于存储获取到的样本值 (Output parameter for storing the obtained sample value)
 * @return 成功返回0，失败返回错误代码 (Returns 0 on success, error code on failure)
 */
  int get_sample_at(const size_t iteration, int64_t & sample) const;

  /**
 * @brief 将结果写入文件 (Write results to file)
 * @return 成功返回0，失败返回错误代码 (Returns 0 on success, error code on failure)
 */
  int write_results();

  /**
 * @brief 将结果写入指定的文件 (Write results to the specified file)
 * @param filename 要写入的文件名 (Filename to write the results to)
 * @return 成功返回0，失败返回错误代码 (Returns 0 on success, error code on failure)
 */
  int write_results_file(char * filename);

  /**
 * @brief 将结果转换为字符串格式 (Convert results to string format)
 * @param name 测试名称 (Test name)
 * @return 结果字符串 (Results string)
 */
  std::string results_to_string(char * name);

  /**
 * @brief 完成测试并释放资源 (Finish the test and release resources)
 * @return 成功返回0，失败返回错误代码 (Returns 0 on success, error code on failure)
 */
  int finish();

  /**
 * @brief 获取测试参数 (Get test parameters)
 * @return 参数结构体指针 (Pointer to the parameters structure)
 */
  struct rttest_params * get_params();

  /**
 * @brief 设置测试参数 (Set test parameters)
 * @param params 参数结构体指针 (Pointer to the parameters structure)
 */
  void set_params(struct rttest_params * params);

  /**
 * @brief 初始化动态内存 (Initialize dynamic memory)
 */
  void initialize_dynamic_memory();
};

// 全局变量，用于追踪线程 (Global variables for tracking threads)
std::map<pthread_t, Rttest> rttest_instance_map;
pthread_t initial_thread_id = 0;

// 构造函数，初始化结果结构体 (Constructor, initializes the result structure)
Rttest::Rttest()
{
  // 将结果结构体中的所有字节设置为0 (Set all bytes in the result structure to 0)
  memset(&this->results, 0, sizeof(struct rttest_results));

  // 初始化最小延迟为 INT_MAX (Initialize minimum latency to INT_MAX)
  this->results.min_latency = INT_MAX;

  // 初始化最大延迟为 INT_MIN (Initialize maximum latency to INT_MIN)
  this->results.max_latency = INT_MIN;
}

// 析构函数 (Destructor)
Rttest::~Rttest() {}

// 函数 (Functions)

/**
 * @brief 设置实时性测试参数 (Set real-time test parameters)
 * 
 * @param params 指向 rttest_params 结构体的指针 (Pointer to the rttest_params structure)
 */
void Rttest::set_params(struct rttest_params * params) { this->params = *params; }

/**
 * @brief 获取实时性测试参数 (Get real-time test parameters)
 * 
 * @return 返回指向 rttest_params 结构体的指针 (Returns a pointer to the rttest_params structure)
 */
struct rttest_params * Rttest::get_params() { return &(this->params); }

/**
 * @brief 记录抖动的函数 (Function to record jitter)
 *
 * @param[in] deadline 截止时间 (Deadline time)
 * @param[in] result_time 实际完成时间 (Actual completion time)
 * @param[in] iteration 迭代次数 (Iteration count)
 * @return int 返回值，0表示成功，-1表示失败 (Return value, 0 for success, -1 for failure)
 */
int Rttest::record_jitter(
  const struct timespec * deadline, const struct timespec * result_time, const size_t iteration)
{
  size_t i = iteration;
  // 检查设置是否允许缓冲区记录 (Check if settings authorize buffer recording)
  if (this->params.iterations == 0) {
    i = 0;
  }
  struct timespec jitter;
  int parity = 1;
  // 如果实际完成时间大于截止时间，说明错过了截止时间 (If the actual completion time is greater than the deadline time, it means the deadline is missed)
  if (timespec_gt(result_time, deadline)) {
    // 错过截止时间 (Missed a deadline)
    subtract_timespecs(result_time, deadline, &jitter);
  } else {
    subtract_timespecs(deadline, result_time, &jitter);
    parity = -1;
  }
  // 记录抖动 (Record jitter)
  if (i >= this->sample_buffer.latency_samples.size()) {
    return -1;
  }
  this->sample_buffer.latency_samples[i] = parity * timespec_to_uint64(&jitter);
  return 0;
}

/**
 * @brief 获取特定线程的Rttest实例 (Get Rttest instance for a specific thread)
 *
 * @param[in] thread_id 线程ID (Thread ID)
 * @return Rttest* 返回Rttest实例指针，如果没有找到则返回NULL (Return pointer to Rttest instance, or NULL if not found)
 */
Rttest * get_rttest_thread_instance(pthread_t thread_id)
{
  // 如果线程ID在实例映射中不存在，则返回NULL (If the thread ID is not in the instance map, return NULL)
  if (rttest_instance_map.count(thread_id) == 0) {
    return NULL;
  }
  // 返回对应线程ID的Rttest实例 (Return the Rttest instance for the corresponding thread ID)
  return &rttest_instance_map[thread_id];
}

/**
 * @brief 解析带单位的大小字符串，返回以字节为单位的值 (Parse a size string with units and return the value in bytes)
 *
 * @param[in] optarg 带有单位的大小字符串 (Size string with units)
 * @return uint64_t 以字节为单位的大小值 (Size value in bytes)
 */
uint64_t rttest_parse_size_units(char * optarg)
{
  uint64_t ret; // 返回值 (Return value)

  std::string input(
    optarg); // 将输入参数转换为 std::string 类型 (Convert input parameter to std::string type)
  std::vector<std::string> tokens = {
    "gb", "mb", "kb", "b"}; // 定义一个单位列表 (Define a list of units)

  // 遍历单位列表 (Iterate through the list of units)
  for (size_t i = 0; i < 4; ++i) {
    size_t idx = input.find(
      tokens
        [i]); // 查找单位在输入字符串中的位置 (Find the position of the unit in the input string)

    // 如果找到单位 (If the unit is found)
    if (idx != std::string::npos) {
      // 计算以字节为单位的大小值并跳出循环 (Calculate the size value in bytes and break out of the loop)
      ret = std::stoll(input.substr(0, idx)) * std::pow(2, (3 - i) * 10);
      break;
    }

    // 如果遍历到最后一个单位且没有找到匹配的单位 (If iterating to the last unit and no matching unit is found)
    if (i == 3) {
      // 默认单位为兆字节 (Default units are megabytes)
      ret = std::stoll(input) * std::pow(2, 20);
    }
  }

  return ret; // 返回以字节为单位的大小值 (Return the size value in bytes)
}

/**
 * @brief 读取命令行参数 (Read command line arguments)
 * @param argc 参数数量 (Number of arguments)
 * @param argv 参数值数组 (Array of argument values)
 * @return 初始化结果 (Initialization result)
 */
int Rttest::read_args(int argc, char ** argv)
{
  // -i,--iterations 迭代次数 (Number of iterations)
  size_t iterations = 1000;
  // -u,--update-period 更新周期 (Update period)
  struct timespec update_period;
  update_period.tv_sec = 0;
  update_period.tv_nsec = 1000000;
  // -t,--thread-priority 线程优先级 (Thread priority)
  int sched_priority = 80;
  // -s,--sched-policy 调度策略 (Scheduling policy)
  size_t sched_policy = SCHED_RR;
  // -m,--memory-size 内存大小 (Memory size)
  size_t stack_size = 1024 * 1024;
  // -d,--prefault-dynamic-memory-size 预分配动态内存大小 (Prefault dynamic memory size)
  uint64_t prefault_dynamic_size = 8589934592UL; // 8GB
  // -f,--filename 文件名 (Filename)
  // 不指定文件名则不写入文件 (Don't write a file unless filename specified)
  char * filename = nullptr;
  int c;

  std::string args_string = "i:u:p:t:s:m:d:f:r:";
  opterr = 0;
  optind = 1;

  // 使用 getopt 解析命令行参数 (Parse command line arguments using getopt)
  while ((c = getopt(argc, argv, args_string.c_str())) != -1) {
    switch (c) {
    case 'i': {
      int arg = atoi(optarg);
      if (arg < 0) {
        iterations = 0;
      } else {
        iterations = arg;
      }
      break;
    }
    case 'u': {
      // 解析时间单位 (Parse time units)
      uint64_t nsec;
      std::string input(optarg);
      std::vector<std::string> tokens = {"ns", "us", "ms", "s"};
      for (size_t i = 0; i < 4; ++i) {
        size_t idx = input.find(tokens[i]);
        if (idx != std::string::npos) {
          nsec = stoull(input.substr(0, idx)) * std::pow(10, i * 3);
          break;
        }
        if (i == 3) {
          // 默认单位是微秒 (Default units are microseconds)
          nsec = stoull(input) * 1000;
        }
      }

      uint64_to_timespec(nsec, &update_period);
    } break;
    case 't':
      sched_priority = atoi(optarg);
      break;
    case 's': {
      std::string input(optarg);
      if (input == "fifo") {
        sched_policy = SCHED_FIFO;
      } else if (input == "rr") {
        sched_policy = SCHED_RR;
      } else {
        fprintf(stderr, "Invalid option entered for scheduling policy: %s\n", input.c_str());
        fprintf(stderr, "Valid options are: fifo, rr\n");
        exit(-1);
      }
    } break;
    case 'm':
      stack_size = rttest_parse_size_units(optarg);
      break;
    case 'd':
      prefault_dynamic_size = rttest_parse_size_units(optarg);
      break;
    case 'f':
      filename = optarg;
      break;
    case '?':
      if (args_string.find(optopt) != std::string::npos) {
        fprintf(stderr, "Option -%c requires an argument.\n", optopt);
      } else if (isprint(optopt)) {
        fprintf(stderr, "Unknown option `-%c'.\n", optopt);
      } else {
        fprintf(stderr, "Unknown option character `\x%x'.\n", optopt);
      }
      break;
    default:
      exit(-1);
    }
  }

  // 使用解析后的参数初始化对象 (Initialize the object with the parsed arguments)
  return this->init(
    iterations, update_period, sched_policy, sched_priority, stack_size, prefault_dynamic_size,
    filename);
}

/**
 * @brief 获取实时测试参数 (Get real-time test parameters)
 *
 * @param params_in 实时测试参数的指针 (Pointer to the real-time test parameters)
 * @return 成功返回0，失败返回-1 (Returns 0 on success, -1 on failure)
 */
int rttest_get_params(struct rttest_params * params_in)
{
  // 检查输入参数是否为空 (Check if the input parameter is NULL)
  if (params_in == NULL) {
    return -1;
  }

  // 获取当前线程的实时测试实例 (Get the real-time test instance of the current thread)
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());

  // 如果实例不存在，则返回-1 (If the instance does not exist, return -1)
  if (!thread_rttest_instance) {
    return -1;
  }

  // 将实例的参数赋值给输入参数 (Assign the instance's parameters to the input parameter)
  *params_in = *thread_rttest_instance->get_params();

  return 0;
}

/**
 * @brief 初始化新线程的实时测试 (Initialize real-time testing for a new thread)
 *
 * @return 成功返回0，失败返回-1 (Returns 0 on success, -1 on failure)
 */
int rttest_init_new_thread()
{
  // 获取当前线程ID (Get the current thread ID)
  auto thread_id = pthread_self();
  // 获取当前线程的实时测试实例 (Get the real-time test instance of the current thread)
  auto thread_rttest_instance = get_rttest_thread_instance(thread_id);

  // 如果实例不存在 (If the instance does not exist)
  if (thread_rttest_instance == nullptr) {
    // 为当前线程创建新的实时测试实例 (Create a new real-time test instance for the current thread)
    rttest_instance_map.emplace(std::make_pair(thread_id, Rttest()));
  } else {
    // 如果实例已存在，输出错误信息并返回-1 (If the instance already exists, output an error message and return -1)
    fprintf(stderr, "rttest instance for %lu already exists!\n", thread_id);
    return -1;
  }

  // 检查初始线程ID是否有效 (Check if the initial thread ID is valid)
  if (initial_thread_id == 0 || rttest_instance_map.count(initial_thread_id) == 0) {
    return -1;
  }

  // 将初始线程的参数设置为当前线程的参数 (Set the parameters of the initial thread to the current thread's parameters)
  rttest_instance_map[thread_id].set_params(rttest_instance_map[initial_thread_id].get_params());
  // 初始化当前线程的动态内存 (Initialize dynamic memory for the current thread)
  rttest_instance_map[thread_id].initialize_dynamic_memory();

  return 0;
}

/**
 * @brief 读取命令行参数并调用当前线程的Rttest实例。（Read command line arguments and call the current thread's Rttest instance.）
 *
 * @param argc 命令行参数个数（Number of command line arguments）
 * @param argv 命令行参数数组（Array of command line arguments）
 * @return 返回读取参数结果（Return the result of reading the parameters）
 */
int rttest_read_args(int argc, char ** argv)
{
  // 获取当前线程ID（Get the current thread ID）
  auto thread_id = pthread_self();
  // 获取与当前线程ID关联的Rttest实例（Get the Rttest instance associated with the current thread ID）
  auto thread_rttest_instance = get_rttest_thread_instance(thread_id);
  if (!thread_rttest_instance) {
    // 为当前线程创建新的Rttest实例（Create a new Rttest instance for this thread）
    rttest_instance_map.emplace(std::make_pair(thread_id, Rttest()));
    // 如果是第一个实例且初始线程ID为0，则设置初始线程ID（If it's the first instance and the initial thread ID is 0, set the initial thread ID）
    if (rttest_instance_map.size() == 1 && initial_thread_id == 0) {
      initial_thread_id = thread_id;
    }
    // 获取新创建的Rttest实例（Get the newly created Rttest instance）
    thread_rttest_instance = &rttest_instance_map[thread_id];
  }
  // 调用Rttest实例的read_args方法处理命令行参数（Call the read_args method of the Rttest instance to process the command line arguments）
  return thread_rttest_instance->read_args(argc, argv);
}

/**
 * @brief 初始化Rttest实例。（Initialize the Rttest instance.）
 *
 * @param iterations 迭代次数（Number of iterations）
 * @param update_period 更新周期（Update period）
 * @param sched_policy 调度策略（Scheduling policy）
 * @param sched_priority 调度优先级（Scheduling priority）
 * @param stack_size 栈大小（Stack size）
 * @param prefault_dynamic_size 预处理动态内存大小（Prefault dynamic memory size）
 * @param filename 保存结果的文件名（Filename to save the results）
 * @return 初始化结果，0表示成功，-1表示失败（Initialization result, 0 means success, -1 means failure）
 */
int Rttest::init(
  size_t iterations,
  struct timespec update_period,
  size_t sched_policy,
  int sched_priority,
  size_t stack_size,
  uint64_t prefault_dynamic_size,
  char * filename)
{
  // 设置参数（Set parameters）
  this->params.iterations = iterations;
  this->params.update_period = update_period;
  this->params.sched_policy = sched_policy;
  this->params.sched_priority = sched_priority;
  this->params.stack_size = stack_size;
  this->params.prefault_dynamic_size = prefault_dynamic_size;

  // 如果filename不为空，则分配内存并复制文件名（If filename is not nullptr, allocate memory and copy the file name）
  if (filename != nullptr) {
    this->params.filename = strdup(filename);
    if (!this->params.filename) {
      fprintf(stderr, "Failed to allocate filename buffer\n");
      return -1;
    }
    fprintf(stderr, "Writing results to file: %s\n", this->params.filename);
  } else {
    this->params.filename = nullptr;
  }

  // 初始化动态内存（Initialize dynamic memory）
  this->initialize_dynamic_memory();
  // 设置运行状态为1（Set running status to 1）
  this->running = 1;
  // 返回初始化成功（Return initialization success）
  return 0;
}

/**
 * @brief 初始化动态内存 (Initialize dynamic memory)
 *
 * @param[in] none
 * @return void
 */
void Rttest::initialize_dynamic_memory()
{
  // 获取迭代次数 (Get the number of iterations)
  size_t iterations = this->params.iterations;

  // 如果迭代次数为0，将其设置为1 (If the number of iterations is 0, set it to 1)
  if (iterations == 0) {
    // 分配一个大小为1的样本缓冲区 (Allocate a sample buffer of size 1)
    iterations = 1;
  }

  // 调整样本缓冲区的大小以适应迭代次数 (Resize the sample buffer to fit the number of iterations)
  this->sample_buffer.resize(iterations);
}

/**
 * @brief 初始化实时测试 (Initialize real-time test)
 *
 * @param[in] iterations 迭代次数 (Number of iterations)
 * @param[in] update_period 更新周期 (Update period)
 * @param[in] sched_policy 调度策略 (Scheduling policy)
 * @param[in] sched_priority 调度优先级 (Scheduling priority)
 * @param[in] stack_size 栈大小 (Stack size)
 * @param[in] prefault_dynamic_size 预故障动态大小 (Prefault dynamic size)
 * @param[in] filename 文件名 (Filename)
 * @return int 返回初始化结果 (Return initialization result)
 */
int rttest_init(
  size_t iterations,
  struct timespec update_period,
  size_t sched_policy,
  int sched_priority,
  size_t stack_size,
  uint64_t prefault_dynamic_size,
  char * filename)
{
  // 获取当前线程ID (Get the current thread ID)
  auto thread_id = pthread_self();

  // 获取当前线程的实时测试实例 (Get the real-time test instance of the current thread)
  auto thread_rttest_instance = get_rttest_thread_instance(thread_id);

  // 如果当前线程没有实时测试实例，则为其创建一个新的实例 (If the current thread does not have a real-time test instance, create a new one for it)
  if (thread_rttest_instance == nullptr) {
    // 创建一个新的Rttest实例对，用于此线程 (Create a new Rttest instance pair for this thread)
    std::pair<pthread_t, Rttest> instance;
    instance.first = thread_id;

    // 将实例添加到实例映射中 (Add the instance to the instance map)
    rttest_instance_map.emplace(instance);

    // 如果实例映射的大小为1且初始线程ID为0，则将当前线程ID设置为初始线程ID (If the size of the instance map is 1 and the initial thread ID is 0, set the current thread ID as the initial thread ID)
    if (rttest_instance_map.size() == 1 && initial_thread_id == 0) {
      initial_thread_id = thread_id;
    }

    // 获取当前线程的实时测试实例 (Get the real-time test instance of the current thread)
    thread_rttest_instance = &rttest_instance_map[thread_id];
  }

  // 调用init方法并返回结果 (Call the init method and return the result)
  return thread_rttest_instance->init(
    iterations, update_period, sched_policy, sched_priority, stack_size, prefault_dynamic_size,
    filename);
}

/**
 * @brief 获取下一个资源使用情况（Get the next resource usage）
 * 
 * @param i 用于确定当前迭代次数的索引（Index used to determine the current iteration count）
 * @return int 成功返回0，失败返回-1（Return 0 on success, -1 on failure）
 */
int Rttest::get_next_rusage(size_t i)
{
  // 让linter跳过这些行，因为getrusage使用long（Have the linter skip these lines because getrusage uses long）
  long prev_maj_pagefaults = this->prev_usage.ru_majflt; // NOLINT
  long prev_min_pagefaults = this->prev_usage.ru_minflt; // NOLINT

  // 获取线程资源使用情况（Get the resource usage of the thread）
  if (getrusage(RUSAGE_THREAD, &this->prev_usage) != 0) {
    return -1;
  }
  // 检查资源使用情况是否合理（Check if the resource usage is reasonable）
  assert(this->prev_usage.ru_majflt >= prev_maj_pagefaults);
  assert(this->prev_usage.ru_minflt >= prev_min_pagefaults);

  // 如果迭代次数为0，则将i设置为0（If the number of iterations is 0, set i to 0）
  if (this->params.iterations == 0) {
    i = 0;
  }

  // 计算主要页面错误和次要页面错误（Calculate major page faults and minor page faults）
  this->sample_buffer.major_pagefaults[i] = this->prev_usage.ru_majflt - prev_maj_pagefaults;
  this->sample_buffer.minor_pagefaults[i] = this->prev_usage.ru_minflt - prev_min_pagefaults;

  return 0;
}

/**
 * @brief 获取下一个资源使用情况的包装函数（Wrapper function for getting the next resource usage）
 * 
 * @param i 用于确定当前迭代次数的索引（Index used to determine the current iteration count）
 * @return int 成功返回0，失败返回-1（Return 0 on success, -1 on failure）
 */
int rttest_get_next_rusage(size_t i)
{
  // 获取当前线程的Rttest实例（Get the Rttest instance of the current thread）
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }

  // 调用Rttest实例的get_next_rusage方法（Call the get_next_rusage method of the Rttest instance）
  return thread_rttest_instance->get_next_rusage(i);
}

/**
 * @brief 执行用户函数并获取资源使用情况（Execute the user function and get the resource usage）
 * 
 * @param user_function 用户提供的函数指针（Pointer to the user-provided function）
 * @param args 用户提供的函数参数（Arguments for the user-provided function）
 * @return int 成功返回0，失败返回-1（Return 0 on success, -1 on failure）
 */
int rttest_spin(void * (*user_function)(void *), void * args)
{
  // 获取当前线程的Rttest实例（Get the Rttest instance of the current thread）
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  if (!thread_rttest_instance) {
    return -1;
  }

  // 调用Rttest实例的spin方法（Call the spin method of the Rttest instance）
  return thread_rttest_instance->spin(user_function, args);
}

/**
 * @brief 在指定周期内执行一次用户函数 (Execute the user function once within the specified period)
 *
 * @param[in] user_function 用户函数指针 (Pointer to the user function)
 * @param[in] args 用户函数参数 (Arguments for the user function)
 * @param[in] start_time 开始时间 (Start time)
 * @param[in] update_period 更新周期 (Update period)
 * @param[in] i 迭代次数 (Iteration count)
 * @return 成功返回0，失败返回-1 (Returns 0 on success, -1 on failure)
 */
int rttest_spin_once_period(
  void * (*user_function)(void *),
  void * args,
  const struct timespec * start_time,
  const struct timespec * update_period,
  const size_t i)
{
  // 获取当前线程的实例 (Get the current thread instance)
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  // 如果没有找到实例，返回-1 (If the instance is not found, return -1)
  if (!thread_rttest_instance) {
    return -1;
  }
  // 调用spin_once方法执行用户函数 (Call the spin_once method to execute the user function)
  return thread_rttest_instance->spin_once(user_function, args, start_time, update_period, i);
}

/**
 * @brief 执行一次用户函数 (Execute the user function once)
 *
 * @param[in] user_function 用户函数指针 (Pointer to the user function)
 * @param[in] args 用户函数参数 (Arguments for the user function)
 * @param[in] start_time 开始时间 (Start time)
 * @param[in] i 迭代次数 (Iteration count)
 * @return 成功返回0，失败返回-1 (Returns 0 on success, -1 on failure)
 */
int rttest_spin_once(
  void * (*user_function)(void *), void * args, const struct timespec * start_time, const size_t i)
{
  // 获取当前线程的实例 (Get the current thread instance)
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  // 如果没有找到实例，返回-1 (If the instance is not found, return -1)
  if (!thread_rttest_instance) {
    return -1;
  }
  // 调用spin_once方法执行用户函数 (Call the spin_once method to execute the user function)
  return thread_rttest_instance->spin_once(user_function, args, start_time, i);
}

/**
 * @brief Rttest类的spin方法，执行用户函数 (The spin method of the Rttest class, which executes the user function)
 *
 * @param[in] user_function 用户函数指针 (Pointer to the user function)
 * @param[in] args 用户函数参数 (Arguments for the user function)
 * @return 成功返回0，失败返回-1 (Returns 0 on success, -1 on failure)
 */
int Rttest::spin(void * (*user_function)(void *), void * args)
{
  // 调用rttest_spin_period方法执行用户函数 (Call the rttest_spin_period method to execute the user function)
  return rttest_spin_period(
    user_function, args, &this->params.update_period, this->params.iterations);
}

/**
 * @brief 以固定频率执行用户函数的方法 (Method for executing user function at a fixed frequency)
 *
 * @param[in] user_function 用户提供的需要周期性执行的函数指针 (Pointer to the user-provided function that needs to be executed periodically)
 * @param[in] args 传递给用户函数的参数 (Arguments passed to the user function)
 * @param[in] update_period 更新周期，表示用户函数执行的时间间隔 (Update period, indicating the time interval for the execution of the user function)
 * @param[in] iterations 执行用户函数的次数，如果为0，则持续执行直到程序停止 (Number of times the user function is executed, if 0, it will continue to execute until the program stops)
 *
 * @return 返回0表示成功，其他值表示错误 (Return 0 indicates success, other values indicate errors)
 */
int Rttest::spin_period(
  void * (*user_function)(void *),
  void * args,
  const struct timespec * update_period,
  const size_t iterations)
{
  // 获取当前时间 (Get the current time)
  struct timespec start_time;
  clock_gettime(CLOCK_MONOTONIC, &start_time);

  // 当iterations为0时，持续执行用户函数，直到程序停止 (When iterations is 0, the user function is executed continuously until the program stops)
  if (iterations == 0) {
    size_t i = 0;
    while (this->running != 0) {
      // 调用 spin_once 执行一次用户函数 (Call spin_once to execute the user function once)
      if (spin_once(user_function, args, &start_time, update_period, i) != 0) {
        // 如果 spin_once 返回错误，抛出异常 (If spin_once returns an error, throw an exception)
        throw std::runtime_error("error in spin_once");
      }
      // 更新计数器 (Update the counter)
      ++i;
    }
  } else {
    // 当iterations不为0时，执行用户函数指定的次数 (When iterations is not 0, execute the user function the specified number of times)
    for (size_t i = 0; i < iterations; i++) {
      // 调用 spin_once 执行一次用户函数 (Call spin_once to execute the user function once)
      if (spin_once(user_function, args, &start_time, update_period, i) != 0) {
        // 如果 spin_once 返回错误，抛出异常 (If spin_once returns an error, throw an exception)
        throw std::runtime_error("error in spin_once");
      }
    }
  }

  // 返回0表示成功 (Return 0 indicates success)
  return 0;
}

/**
 * @brief 执行一次实时任务的封装函数（Wrapper function to execute the real-time task once）
 * 
 * @param user_function 用户定义的实时任务函数指针（Pointer to the user-defined real-time task function）
 * @param args 用户定义的实时任务函数参数（Arguments for the user-defined real-time task function）
 * @param start_time 任务开始时间（Task start time）
 * @param i 当前迭代次数（Current iteration count）
 * @return int 成功返回0，失败返回-1（Returns 0 on success, -1 on failure）
 */
int Rttest::spin_once(
  void * (*user_function)(void *), void * args, const struct timespec * start_time, const size_t i)
{
  // 调用另一个 spin_once 函数重载版本，并传入更新周期参数
  // Call another overloaded version of spin_once and pass in the update period parameter
  return this->spin_once(user_function, args, start_time, &this->params.update_period, i);
}

/**
 * @brief 执行一次实时任务（Execute the real-time task once）
 * 
 * @param user_function 用户定义的实时任务函数指针（Pointer to the user-defined real-time task function）
 * @param args 用户定义的实时任务函数参数（Arguments for the user-defined real-time task function）
 * @param start_time 任务开始时间（Task start time）
 * @param update_period 任务更新周期（Task update period）
 * @param i 当前迭代次数（Current iteration count）
 * @return int 成功返回0，失败返回-1（Returns 0 on success, -1 on failure）
 */
int Rttest::spin_once(
  void * (*user_function)(void *),
  void * args,
  const struct timespec * start_time,
  const struct timespec * update_period,
  const size_t i)
{
  // 检查输入参数的有效性
  // Check the validity of input parameters
  if (!start_time || !update_period || (i > params.iterations && params.iterations > 0)) {
    return -1;
  }
  // 如果是第一次迭代，获取线程的资源使用情况
  // If it's the first iteration, get the resource usage of the thread
  if (i == 0) {
    if (getrusage(RUSAGE_THREAD, &this->prev_usage) != 0) {
      return -1;
    }
    printf("Initial major pagefaults: %ld\n", this->prev_usage.ru_majflt);
    printf("Initial minor pagefaults: %ld\n", this->prev_usage.ru_minflt);
  }
  // 初始化时间变量
  // Initialize time variables
  struct timespec wakeup_time, current_time;
  // 计算唤醒时间
  // Calculate the wakeup time
  multiply_timespec(update_period, i, &wakeup_time);
  add_timespecs(start_time, &wakeup_time, &wakeup_time);
  // 等待唤醒时间到达
  // Wait for the wakeup time to arrive
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
  // 获取当前时间
  // Get the current time
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  // 记录抖动（jitter）信息
  // Record jitter information
  this->record_jitter(&wakeup_time, &current_time, i);

  // 调用用户定义的实时任务函数
  // Call the user-defined real-time task function
  user_function(args);
  // 获取下一次资源使用情况
  // Get the next resource usage
  this->get_next_rusage(i);
  // 累积统计信息
  // Accumulate statistical information
  this->accumulate_statistics(i);
  return 0;
}

/**
 * @brief 以固定周期执行用户函数的方法
 * @param user_function 用户提供的函数指针，该函数接收一个void* 类型参数并返回一个void* 类型值
 * @param args 传递给用户函数的参数
 * @param update_period 更新周期（时间间隔），用于控制用户函数执行的频率
 * @param iterations 需要执行用户函数的次数
 * @return 成功返回0，失败返回-1
 *
 * @brief A method to execute a user function at a fixed period
 * @param user_function A function pointer provided by the user, which takes a void* type parameter and returns a void* type value
 * @param args The arguments passed to the user function
 * @param update_period The update period (time interval) for controlling the frequency of user function execution
 * @param iterations The number of times the user function needs to be executed
 * @return Returns 0 on success, -1 on failure
 */
int rttest_spin_period(
  void * (*user_function)(void *),
  void * args,
  const struct timespec * update_period,
  const size_t iterations)
{
  // 获取当前线程的实例
  // Get the current thread instance
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  // 如果线程实例不存在，则返回-1
  // If the thread instance does not exist, return -1
  if (!thread_rttest_instance) {
    return -1;
  }
  // 调用spin_period方法，并返回结果
  // Call the spin_period method and return the result
  return thread_rttest_instance->spin_period(user_function, args, update_period, iterations);
}

/**
 * @brief 锁定内存的方法
 * @return 成功返回0，失败返回-1
 *
 * @brief A method to lock memory
 * @return Returns 0 on success, -1 on failure
 */
int rttest_lock_memory()
{
  // 获取当前线程的实例
  // Get the current thread instance
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  // 如果线程实例不存在，则返回-1
  // If the thread instance does not exist, return -1
  if (!thread_rttest_instance) {
    return -1;
  }
  // 调用lock_memory方法，并返回结果
  // Call the lock_memory method and return the result
  return thread_rttest_instance->lock_memory();
}

/**
 * @brief 实际执行锁定内存操作的方法
 * @return 成功返回0，失败返回错误码
 *
 * @brief The method that actually performs the memory locking operation
 * @return Returns 0 on success, error code on failure
 */
int Rttest::lock_memory() { return mlockall(MCL_CURRENT | MCL_FUTURE); }

/**
 * @brief 锁定并预留动态内存的方法
 * @return 成功返回0，失败返回-1
 *
 * @brief A method to lock and reserve dynamic memory
 * @return Returns 0 on success, -1 on failure
 */
int rttest_lock_and_prefault_dynamic()
{
  // 获取当前线程的实例
  // Get the current thread instance
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  // 如果线程实例不存在，则返回-1
  // If the thread instance does not exist, return -1
  if (!thread_rttest_instance) {
    return -1;
  }
  // 调用lock_and_prefault_dynamic方法，并返回结果
  // Call the lock_and_prefault_dynamic method and return the result
  return thread_rttest_instance->lock_and_prefault_dynamic();
}

/**
 * @brief 对动态内存进行锁定和预处理 (Lock and prefault dynamic memory)
 *
 * @return 成功时返回 0，失败时返回 -1 (Return 0 on success, -1 on failure)
 */
int Rttest::lock_and_prefault_dynamic()
{
  // 尝试锁定进程的当前和将来分配的内存 (Attempt to lock the process's current and future allocated memory)
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    perror("mlockall failed");
    return -1;
  }

  // 关闭 malloc 的内存整理功能 (Turn off malloc trimming)
  if (mallopt(M_TRIM_THRESHOLD, -1) == 0) {
    perror("mallopt for trim threshold failed");
    munlockall();
    return -1;
  }

  // 关闭 mmap 的使用 (Turn off mmap usage)
  if (mallopt(M_MMAP_MAX, 0) == 0) {
    perror("mallopt for mmap failed");
    mallopt(M_TRIM_THRESHOLD, 128 * 1024);
    munlockall();
    return -1;
  }

  struct rusage
    usage; // 存储资源使用统计信息的结构体 (Structure to store resource usage statistics)
  size_t page_size = sysconf(_SC_PAGESIZE); // 获取系统页面大小 (Get system page size)
  getrusage(
    RUSAGE_SELF, &usage); // 获取当前进程的资源使用情况 (Get resource usage of the current process)
  size_t prev_minflts =
    usage.ru_minflt; // 上次记录的缺页中断数量 (Number of minor page faults recorded last time)
  size_t prev_majflts =
    usage.ru_majflt; // 上次记录的主要缺页中断数量 (Number of major page faults recorded last time)
  size_t encountered_minflts = 1; // 遇到的缺页中断数量 (Number of minor page faults encountered)
  size_t encountered_majflts =
    1; // 遇到的主要缺页中断数量 (Number of major page faults encountered)

  size_t array_size =
    sizeof(char) * 64 * page_size; // 分配内存块的大小 (Size of memory blocks to allocate)
  size_t total_size = 0;           // 已分配的总内存大小 (Total allocated memory size)
  uint64_t max_size =
    this->params.prefault_dynamic_size; // 最大预处理动态内存大小 (Max prefault dynamic memory size)
  std::vector<char *>
    prefaulter; // 存储分配的内存块指针的向量 (Vector to store pointers to allocated memory blocks)
  prefaulter.reserve(static_cast<size_t>(max_size / array_size));

  // 当没有遇到缺页中断时进行预处理 (Prefault until no more page faults are encountered)
  while (encountered_minflts > 0 || encountered_majflts > 0) {
    char * ptr; // 分配的内存块指针 (Pointer to the allocated memory block)
    try {
      ptr = new char[array_size];
      memset(ptr, 0, array_size);
      total_size += array_size;
    } catch (const std::bad_alloc & e) {
      fprintf(stderr, "Caught exception: %s\n", e.what());
      fprintf(stderr, "Unlocking memory and continuing.\n");
      for (auto & ptr : prefaulter) {
        delete[] ptr;
      }

      mallopt(M_TRIM_THRESHOLD, 128 * 1024);
      mallopt(M_MMAP_MAX, 65536);
      munlockall();
      return -1;
    }

    // 如果已达到最大预处理大小，则删除创建的 char 数组 (If max_size is reached, delete the created char array)
    // 这将防止下次分配时发生缺页中断 (This will prevent pagefault on next allocation)
    if (total_size >= max_size) {
      delete[] ptr;
    } else {
      prefaulter.push_back(ptr);
    }

    getrusage(
      RUSAGE_SELF,
      &usage); // 获取当前进程的资源使用情况 (Get resource usage of the current process)
    size_t current_minflt =
      usage.ru_minflt; // 当前记录的缺页中断数量 (Number of minor page faults recorded currently)
    size_t current_majflt =
      usage
        .ru_majflt; // 当前记录的主要缺页中断数量 (Number of major page faults recorded currently)
    encountered_minflts = current_minflt - prev_minflts;
    encountered_majflts = current_majflt - prev_majflts;
    prev_minflts = current_minflt;
    prev_majflts = current_majflt;
  }

  // 删除所有分配的内存块 (Delete all allocated memory blocks)
  for (auto & ptr : prefaulter) {
    delete[] ptr;
  }
  return 0;
}

/**
 * @brief 预先分配栈内存并初始化为0 (Prefault a stack of given size and initialize it to 0)
 * @param stack_size 栈大小（Stack size）
 * @return 总是返回0 (Always returns 0)
 */
int rttest_prefault_stack_size(const size_t stack_size)
{
  // 使用alloca分配栈内存，并将其类型转换为unsigned char指针
  // Allocate stack memory using alloca and cast it to unsigned char pointer
  unsigned char * stack = static_cast<unsigned char *>(alloca(stack_size));

  // 将栈内存设置为0
  // Set the stack memory to 0
  memset(stack, 0, stack_size);

  return 0;
}

/**
 * @brief 预先分配当前线程的栈内存 (Prefault the stack for the current thread)
 * @return 成功时返回0，失败时返回-1 (Returns 0 on success and -1 on failure)
 */
int rttest_prefault_stack()
{
  // 获取当前线程的rttest实例
  // Get the rttest instance for the current thread
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());

  // 如果没有找到实例，则返回-1
  // Return -1 if no instance is found
  if (!thread_rttest_instance) {
    return -1;
  }

  // 预先分配栈内存并初始化为0
  // Prefault stack memory and initialize it to 0
  return rttest_prefault_stack_size(thread_rttest_instance->get_params()->stack_size);
}

/**
 * @brief 设置当前线程的默认优先级 (Set the default priority for the current thread)
 * @return 成功时返回0，失败时返回-1 (Returns 0 on success and -1 on failure)
 */
int rttest_set_thread_default_priority()
{
  // 获取当前线程的rttest实例
  // Get the rttest instance for the current thread
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());

  // 如果没有找到实例，则返回-1
  // Return -1 if no instance is found
  if (!thread_rttest_instance) {
    return -1;
  }

  // 设置调度优先级和策略
  // Set the scheduling priority and policy
  return rttest_set_sched_priority(
    thread_rttest_instance->get_params()->sched_priority,
    thread_rttest_instance->get_params()->sched_policy);
}

/**
 * @brief 设置调度优先级 (Set the scheduling priority)
 * @param sched_priority 调度优先级（Scheduling priority）
 * @param policy 调度策略（Scheduling policy）
 * @return 成功时返回0，失败时返回错误代码 (Returns 0 on success and error code on failure)
 */
int rttest_set_sched_priority(size_t sched_priority, int policy)
{
  // 定义sched_param结构体
  // Define sched_param structure
  struct sched_param param;

  // 设置调度优先级
  // Set the scheduling priority
  param.sched_priority = sched_priority;

  // 使用sched_setscheduler设置任意进程的优先级
  // Set the priority of an arbitrary process using sched_setscheduler
  return sched_setscheduler(0, policy, &param);
}

/**
 * @brief 以支持 doxygen 的形式添加参数列表的说明 Accumulate the statistics of the current iteration.
 * @param iteration The current iteration number
 * @return 0 on success, -1 on error
 *
 * @brief 累积当前迭代的统计数据
 * @param iteration 当前迭代次数
 * @return 成功时返回0，错误时返回-1
 */
int Rttest::accumulate_statistics(size_t iteration)
{
  size_t i = iteration; // 设置 i 等于传入的迭代次数 (Set i equal to the passed iteration)

  this->results.iteration =
    iteration; // 将结果的迭代次数设置为传入的迭代次数 (Set the results' iteration to the passed iteration)

  if (params.iterations == 0) { // 如果参数中的迭代次数为0 (If the iterations in params is 0)
    i = 0;                      // 将 i 设置为0 (Set i to 0)
  } else if (
    iteration >
    params
      .iterations) { // 否则，如果传入的迭代次数大于参数中的迭代次数 (Otherwise, if the passed iteration is greater than the iterations in params)
    return -1;       // 返回 -1 表示错误 (Return -1 indicating an error)
  }

  int64_t latency = sample_buffer.latency_samples[i]; // 获取延迟样本 (Get the latency sample)

  // 更新最大延迟 (Update the max latency)
  if (latency > this->results.max_latency) {
    this->results.max_latency = latency;
  }

  // 更新最小延迟 (Update the min latency)
  if (latency < this->results.min_latency) {
    this->results.min_latency = latency;
  }

  if (iteration > 0) {
    // 累积平均延迟 (Accumulate the mean latency)
    this->results.mean_latency =
      this->results.mean_latency +
      (sample_buffer.latency_samples[i] - this->results.mean_latency) / (iteration + 1);
  } else {
    // 初始化平均延迟 (Initialize the mean latency)
    this->results.mean_latency = sample_buffer.latency_samples[i];
  }

  // 累积次要页面错误 (Accumulate minor page faults)
  this->results.minor_pagefaults += sample_buffer.minor_pagefaults[i];

  // 累积主要页面错误 (Accumulate major page faults)
  this->results.major_pagefaults += sample_buffer.major_pagefaults[i];

  this->results_initialized = true; // 设置结果已初始化标志 (Set the results initialized flag)
  return 0;                         // 返回0表示成功 (Return 0 indicating success)
}

/**
 * @brief Calculate the overall statistics and store them in the output struct.
 * @param output The rttest_results struct to store the calculated statistics
 * @return 0 on success, -1 on error
 *
 * @brief 计算整体统计数据并将其存储在输出结构中
 * @param output 存储计算出的统计数据的 rttest_results 结构
 * @return 成功时返回0，错误时返回-1
 */
int Rttest::calculate_statistics(struct rttest_results * output)
{
  if (output == NULL) { // 如果输出为空 (If the output is NULL)
    fprintf(
      stderr, "Need to allocate rttest_results struct\n"); // 打印错误信息 (Print an error message)
    return -1; // 返回 -1 表示错误 (Return -1 indicating an error)
  }

  // 计算最小延迟 (Calculate the min latency)
  output->min_latency = *std::min_element(
    this->sample_buffer.latency_samples.begin(), this->sample_buffer.latency_samples.end());

  // 计算最大延迟 (Calculate the max latency)
  output->max_latency = *std::max_element(
    this->sample_buffer.latency_samples.begin(), this->sample_buffer.latency_samples.end());

  // 计算平均延迟 (Calculate the mean latency)
  output->mean_latency =
    std::accumulate(
      this->sample_buffer.latency_samples.begin(), this->sample_buffer.latency_samples.end(), 0.0) /
    this->sample_buffer.latency_samples.size();

  // 计算标准差并尝试避免溢出 (Calculate standard deviation and try to avoid overflow)
  output->latency_stddev = calculate_stddev(this->sample_buffer.latency_samples);

  // 计算次要页面错误 (Calculate minor page faults)
  output->minor_pagefaults = std::accumulate(
    this->sample_buffer.minor_pagefaults.begin(), this->sample_buffer.minor_pagefaults.end(), 0);

  // 计算主要页面错误 (Calculate major page faults)
  output->major_pagefaults = std::accumulate(
    this->sample_buffer.major_pagefaults.begin(), this->sample_buffer.major_pagefaults.end(), 0);

  return 0; // 返回0表示成功 (Return 0 indicating success)
}

/**
 * @brief 计算实时测试结果的统计信息 (Calculate the statistics of real-time test results)
 *
 * @param[in] results 结果数据结构指针，用于存储计算出的统计信息 (Pointer to the result data structure for storing the calculated statistics)
 * @return 成功返回0，否则返回-1 (Returns 0 on success, -1 otherwise)
 */
int rttest_calculate_statistics(struct rttest_results * results)
{
  // 获取当前线程的实时测试实例 (Get the real-time test instance of the current thread)
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  // 如果没有找到实例，则返回-1 (If no instance is found, return -1)
  if (!thread_rttest_instance) {
    return -1;
  }
  // 计算统计信息并将结果存储在传入的results参数中 (Calculate the statistics and store the results in the passed-in results parameter)
  return thread_rttest_instance->calculate_statistics(results);
}

/**
 * @brief 获取实时测试结果的统计信息 (Get the statistics of real-time test results)
 *
 * @param[out] output 用于存储统计信息的结果数据结构指针 (Pointer to the result data structure for storing the statistics)
 * @return 成功返回0，否则返回-1 (Returns 0 on success, -1 otherwise)
 */
int rttest_get_statistics(struct rttest_results * output)
{
  // 检查输出参数是否为空 (Check if the output parameter is NULL)
  if (output == NULL) {
    return -1;
  }

  // 获取当前线程的实时测试实例 (Get the real-time test instance of the current thread)
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  // 如果没有找到实例，则返回-1 (If no instance is found, return -1)
  if (!thread_rttest_instance) {
    return -1;
  }
  // 如果结果未初始化，则返回-1 (If the results are not initialized, return -1)
  if (!thread_rttest_instance->results_initialized) {
    return -1;
  }

  // 将计算出的统计信息复制到输出参数中 (Copy the calculated statistics to the output parameter)
  *output = thread_rttest_instance->results;

  return 0;
}

/**
 * @brief 获取指定迭代次数的样本数据 (Get the sample data for the specified number of iterations)
 *
 * @param[in] iteration 迭代次数 (Number of iterations)
 * @param[out] sample 存储样本数据的引用 (Reference to store the sample data)
 * @return 成功返回0，否则返回-1 (Returns 0 on success, -1 otherwise)
 */
int Rttest::get_sample_at(const size_t iteration, int64_t & sample) const
{
  // 如果迭代次数为0，返回第一个样本数据 (If the number of iterations is 0, return the first sample data)
  if (this->params.iterations == 0) {
    sample = this->sample_buffer.latency_samples[0];
    return 0;
  }
  // 如果迭代次数小于总迭代次数，返回对应的样本数据 (If the number of iterations is less than the total number of iterations, return the corresponding sample data)
  if (iteration < this->params.iterations) {
    sample = this->sample_buffer.latency_samples[iteration];
    return 0;
  }
  // 否则返回-1 (Otherwise, return -1)
  return -1;
}

/**
 * @brief 获取指定迭代次数的样本数据 (Get the sample data for the specified number of iterations)
 *
 * @param[in] iteration 迭代次数 (Number of iterations)
 * @param[out] sample 存储样本数据的指针 (Pointer to store the sample data)
 * @return 成功返回0，否则返回-1 (Returns 0 on success, -1 otherwise)
 */
int rttest_get_sample_at(const size_t iteration, int64_t * sample)
{
  // 获取当前线程的实时测试实例 (Get the real-time test instance of the current thread)
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  // 如果没有找到实例，则返回-1 (If no instance is found, return -1)
  if (!thread_rttest_instance) {
    return -1;
  }
  // 如果样本指针为空，则返回-1 (If the sample pointer is NULL, return -1)
  if (sample == NULL) {
    return -1;
  }
  // 获取指定迭代次数的样本数据并存储在传入的sample参数中 (Get the sample data for the specified number of iterations and store it in the passed-in sample parameter)
  return thread_rttest_instance->get_sample_at(iteration, *sample);
}

/**
 * @brief 将实时测试结果转换为字符串 (Converts real-time test results to a string)
 *
 * @param name 测试名称（可选）(Test name (optional))
 * @return std::string 结果字符串 (Result string)
 */
std::string Rttest::results_to_string(char * name)
{
  // 创建一个字符串流对象 (Create a stringstream object)
  std::stringstream sstring;

  // 添加固定的标题到字符串流 (Add fixed title to the stringstream)
  sstring << std::fixed << "rttest statistics";
  // 如果提供了名称，将其添加到字符串流 (If a name is provided, add it to the stringstream)
  if (name != NULL) {
    sstring << " for " << name << ":" << std::endl;
  } else {
    sstring << ":" << std::endl;
  }
  // 添加统计信息到字符串流 (Add statistics to the stringstream)
  sstring << "  - Minor pagefaults: " << results.minor_pagefaults << std::endl;
  sstring << "  - Major pagefaults: " << results.major_pagefaults << std::endl;
  sstring << "  Latency (time after deadline was missed):" << std::endl;
  sstring << "    - Min: " << results.min_latency << " ns" << std::endl;
  sstring << "    - Max: " << results.max_latency << " ns" << std::endl;
  sstring << "    - Mean: " << results.mean_latency << " ns" << std::endl;
  sstring << "    - Standard deviation: " << results.latency_stddev << std::endl;
  // 添加空行 (Add an empty line)
  sstring << std::endl;

  // 返回生成的字符串 (Return the generated string)
  return sstring.str();
}

/**
 * @brief 结束实时测试并获取状态 (Finish the real-time test and get the status)
 *
 * @return int 状态值，-1 表示失败 (Status value, -1 indicates failure)
 */
int rttest_finish()
{
  // 获取当前线程的实时测试实例 (Get the real-time test instance of the current thread)
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
  // 如果没有找到实例，返回 -1 (If no instance is found, return -1)
  if (!thread_rttest_instance) {
    return -1;
  }
  // 调用 finish 方法并获取状态 (Call the finish method and get the status)
  int status = thread_rttest_instance->finish();

  // 从实例映射中删除当前线程的实例 (Remove the instance of the current thread from the instance map)
  rttest_instance_map.erase(pthread_self());

  // 返回状态值 (Return the status value)
  return status;
}

/**
 * @brief 结束实时测试 (Finish the real-time test)
 *
 * @return int 返回值为0表示成功，其他值表示失败 (Return 0 on success, other values indicate failure)
 */
int Rttest::finish()
{
  this->running =
    0; // 将运行状态设置为0，表示不再运行 (Set the running state to 0, indicating it is no longer running)
  munlockall(); // 解锁所有内存页，以允许它们被交换出内存 (Unlock all memory pages, allowing them to be swapped out of memory)

  // Print statistics to screen
  this->calculate_statistics(
    &this
       ->results); // 计算统计结果并存储在results中 (Calculate statistical results and store them in results)
  printf(
    "%s\n", this->results_to_string(this->params.filename)
              .c_str()); // 打印统计结果到屏幕 (Print the statistical results to the screen)
  free(this->params.filename); // 释放文件名内存 (Free the memory of the filename)

  return 0;
}

/**
 * @brief 将实时测试结果写入文件 (Write the real-time test results to a file)
 *
 * @param[in] filename 文件名 (File name)
 * @return int 返回值为0表示成功，其他值表示失败 (Return 0 on success, other values indicate failure)
 */
int rttest_write_results_file(char * filename)
{
  auto thread_rttest_instance = get_rttest_thread_instance(
    pthread_self()); // 获取当前线程的实时测试实例 (Get the real-time test instance of the current thread)
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance->write_results_file(
    filename); // 将实时测试结果写入文件 (Write the real-time test results to the file)
}

/**
 * @brief 将实时测试结果写入默认文件 (Write the real-time test results to the default file)
 *
 * @return int 返回值为0表示成功，其他值表示失败 (Return 0 on success, other values indicate failure)
 */
int rttest_write_results()
{
  auto thread_rttest_instance = get_rttest_thread_instance(
    pthread_self()); // 获取当前线程的实时测试实例 (Get the real-time test instance of the current thread)
  if (!thread_rttest_instance) {
    return -1;
  }
  return thread_rttest_instance
    ->write_results(); // 将实时测试结果写入默认文件 (Write the real-time test results to the default file)
}

/**
 * @brief 写入测试结果 (Write test results)
 *
 * @return 返回写入结果文件的状态 (Return the status of writing to the result file)
 */
int Rttest::write_results()
{
  // 调用 write_results_file 函数，将结果写入指定文件名的文件中 (Call write_results_file function to write the results into the file with the specified filename)
  return this->write_results_file(this->params.filename);
}

/**
 * @brief 将测试结果写入指定文件 (Write test results to the specified file)
 *
 * @param[in] filename 要写入结果的文件名 (The filename of the file to write the results to)
 * @return 返回写入结果的状态 (Return the status of writing the results)
 */
int Rttest::write_results_file(char * filename)
{
  // 如果没有迭代次数，则不保存样本缓冲区，也不写入结果 (If there are no iterations, do not save the sample buffer and do not write the results)
  if (this->params.iterations == 0) {
    fprintf(stderr, "No sample buffer was saved, not writing results\n");
    return -1;
  }
  // 如果未给出结果文件名，则不写入结果 (If no results filename is given, do not write the results)
  if (filename == NULL) {
    fprintf(stderr, "No results filename given, not writing results\n");
    return -1;
  }

  // 打开文件以写入数据 (Open the file for writing data)
  std::ofstream fstream(filename, std::ios::out);

  // 如果无法打开文件，则不写入结果 (If the file cannot be opened, do not write the results)
  if (!fstream.is_open()) {
    fprintf(stderr, "Couldn't open file %s, not writing results\n", filename);
    return -1;
  }

  // 写入表头 (Write the table header)
  fstream << "iteration timestamp latency minor_pagefaults major_pagefaults" << std::endl;

  // 遍历样本缓冲区中的延迟样本，并将每个样本的信息写入文件 (Iterate through the latency samples in the sample buffer and write the information of each sample to the file)
  for (size_t i = 0; i < this->sample_buffer.latency_samples.size(); ++i) {
    fstream << i << " " << timespec_to_uint64(&this->params.update_period) * i << " "
            << this->sample_buffer.latency_samples[i] << " "
            << this->sample_buffer.minor_pagefaults[i] << " "
            << this->sample_buffer.major_pagefaults[i] << std::endl;
  }

  // 关闭文件流 (Close the file stream)
  fstream.close();

  // 返回成功状态 (Return success status)
  return 0;
}

/**
 * @brief 检查 rttest 是否正在运行 (Check if rttest is running)
 *
 * @return 如果 rttest 正在运行，返回 1；否则返回 0 (Return 1 if rttest is running, otherwise return 0)
 */
int rttest_running()
{
  // 获取当前线程的 rttest 实例 (Get the rttest instance of the current thread)
  auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());

  // 如果没有找到实例，则返回 0 (If no instance is found, return 0)
  if (!thread_rttest_instance) {
    return 0;
  }

  // 返回 rttest 实例的运行状态 (Return the running status of the rttest instance)
  return thread_rttest_instance->running;
}
