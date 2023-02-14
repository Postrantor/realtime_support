// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RTTEST__MATH_UTILS_HPP_
#define RTTEST__MATH_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

// 计算给定容器中元素的标准差 (Calculate the standard deviation of elements in a given container)
template <typename container>
double calculate_stddev(const container & vec)
{
  double n = vec.size();  // 获取容器大小 (Get the size of the container)

  // 计算平均值 (Calculate the mean)
  double mean = std::accumulate(vec.begin(), vec.end(), 0.0) / n;

  std::vector<double> diff(n);
  
  // 计算每个元素与平均值的差值 (Calculate the difference between each element and the mean)
  std::transform(
    vec.begin(), vec.end(), diff.begin(), [mean](auto x) -> double { return x - mean; });

  // 首先除以 sqrt(n) (First divide by sqrt(n))
  std::vector<double> div(n);
  std::transform(
    diff.begin(), diff.end(), div.begin(), [n](auto x) -> double { return x / std::sqrt(n); });

  // 求内积 (Calculate the inner product)
  double sq_sum = std::inner_product(div.begin(), div.end(), div.begin(), 0.0);

  // 返回标准差 (Return the standard deviation)
  return std::sqrt(sq_sum);
}

#endif // RTTEST__MATH_UTILS_HPP_
