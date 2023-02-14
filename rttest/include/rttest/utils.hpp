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

#ifndef RTTEST__UTILS_HPP_
#define RTTEST__UTILS_HPP_

#include <stdint.h>
#include <time.h>

#define NSEC_PER_SEC 1000000000 //定义纳秒每秒的值（Define the value of nanoseconds per second）

/**
 * \brief 比较两个 timespec 结构体的时间大小（Compare the time size of two timespec structures）
 * \param t1 第一个 timespec 结构体指针（Pointer to the first timespec structure）
 * \param t2 第二个 timespec 结构体指针（Pointer to the second timespec structure）
 * \return 如果 t1 大于 t2，返回 true；否则返回 false（Return true if t1 is greater than t2, otherwise return false）
 */
static inline bool timespec_gt(const struct timespec * t1, const struct timespec * t2)
{
  if (t1->tv_sec > t2->tv_sec) { //比较秒数（Compare seconds）
    return true;
  }
  if (t1->tv_sec < t2->tv_sec) { //比较秒数（Compare seconds）
    return false;
  }
  return t1->tv_nsec > t2->tv_nsec; //比较纳秒数（Compare nanoseconds）
}

/**
 * \brief 规范化 timespec 结构体（Normalize a timespec structure）
 * \param t 要规范化的 timespec 结构体指针（Pointer to the timespec structure to be normalized）
 */
static inline void normalize_timespec(struct timespec * t)
{
  // TODO(jacquelinekay) improve
  while (t->tv_nsec >= NSEC_PER_SEC) { //确保纳秒小于等于每秒的纳秒数（Ensure nanoseconds are less than or equal to nanoseconds per second）
    t->tv_nsec -= NSEC_PER_SEC;
    t->tv_sec++;
  }
}

/**
 * \brief 将两个 timespec 结构体相加（Add two timespec structures）
 * \param t1 第一个 timespec 结构体指针（Pointer to the first timespec structure）
 * \param t2 第二个 timespec 结构体指针（Pointer to the second timespec structure）
 * \param dst 相加结果的 timespec 结构体指针（Pointer to the timespec structure for the addition result）
 */
static inline void
add_timespecs(const struct timespec * t1, const struct timespec * t2, struct timespec * dst)
{
  dst->tv_sec = t1->tv_sec + t2->tv_sec; //相加秒数（Add seconds）
  dst->tv_nsec = t1->tv_nsec + t2->tv_nsec; //相加纳秒数（Add nanoseconds）
  normalize_timespec(dst); //规范化结果（Normalize the result）
}

/**
 * \brief 计算两个 timespec 结构体的时间差（Calculate the time difference between two timespec structures）
 * \param t1 第一个 timespec 结构体指针（Pointer to the first timespec structure）
 * \param t2 第二个 timespec 结构体指针（Pointer to the second timespec structure）
 * \param dst 时间差的 timespec 结构体指针（Pointer to the timespec structure for the time difference）
 * \return 如果 t1 大于等于 t2，返回 true；否则返回 false（Return true if t1 is greater than or equal to t2, otherwise return false）
 */
static inline bool
subtract_timespecs(const struct timespec * t1, const struct timespec * t2, struct timespec * dst)
{
  if (timespec_gt(t2, t1)) { //判断 t2 是否大于 t1（Check if t2 is greater than t1）
    return subtract_timespecs(t2, t1, dst); //交换顺序重新计算（Recalculate with swapped order）
  }

  dst->tv_sec = t1->tv_sec - t2->tv_sec; //计算秒数差（Calculate the difference in seconds）
  dst->tv_nsec = t1->tv_nsec - t2->tv_nsec; //计算纳秒数差（Calculate the difference in nanoseconds）

  normalize_timespec(dst); //规范化结果（Normalize the result）
  return true;
}

/**
 * @brief 将 timespec 结构体转换为 uint64_t 类型的纳秒值 (Convert timespec structure to uint64_t type nanoseconds)
 *
 * @param[in] t 指向 timespec 结构体的指针 (Pointer to timespec structure)
 * @return uint64_t 转换后的纳秒值 (Converted nanosecond value)
 */
static inline uint64_t timespec_to_uint64(const struct timespec * t)
{
  // 将 timespec 结构体中的秒数和纳秒数分别转换为 uint64_t 类型，并计算总纳秒数
  // (Convert the seconds and nanoseconds in timespec structure to uint64_t type, and calculate total nanoseconds)
  return static_cast<uint64_t>(t->tv_sec) * NSEC_PER_SEC + static_cast<uint64_t>(t->tv_nsec);
}

/**
 * @brief 将 uint64_t 类型的纳秒值转换为 timespec 结构体 (Convert uint64_t type nanoseconds to timespec structure)
 *
 * @param[in] input 输入的纳秒值 (Input nanosecond value)
 * @param[out] t 输出的 timespec 结构体指针 (Output pointer to timespec structure)
 */
static inline void uint64_to_timespec(const uint64_t input, struct timespec * t)
{
  // 计算输入纳秒值对应的秒数和纳秒数
  // (Calculate the corresponding seconds and nanoseconds for the input nanosecond value)
  uint64_t nsecs = input % NSEC_PER_SEC;
  uint64_t secs = (input - nsecs) / NSEC_PER_SEC;

  // 将计算得到的秒数和纳秒数赋值给 timespec 结构体
  // (Assign the calculated seconds and nanoseconds to timespec structure)
  t->tv_sec = static_cast<time_t>(secs);
  t->tv_nsec = static_cast<long>(nsecs); // NOLINT for C type long
}

/**
 * @brief 将 timespec 结构体的时间值乘以一个整数，并将结果存储在另一个 timespec 结构体中
 *        (Multiply the time value of timespec structure by an integer, and store the result in another timespec structure)
 *
 * @param[in] t 输入的 timespec 结构体指针 (Input pointer to timespec structure)
 * @param[in] i 乘法因子 (Multiplication factor)
 * @param[out] result 输出的 timespec 结构体指针，用于存储计算结果 (Output pointer to timespec structure for storing calculation results)
 */
static inline void
multiply_timespec(const struct timespec * t, const uint32_t i, struct timespec * result)
{
  // 将 timespec 结构体转换为 uint64_t 类型的纳秒值，并进行乘法运算
  // (Convert timespec structure to uint64_t type nanoseconds and perform multiplication operation)
  uint64_t result_nsec = i * timespec_to_uint64(t);

  // 将计算得到的纳秒值转换回 timespec 结构体并存储在 result 中
  // (Convert the calculated nanosecond value back to timespec structure and store it in result)
  uint64_to_timespec(result_nsec, result);
}

#endif // RTTEST__UTILS_HPP_
