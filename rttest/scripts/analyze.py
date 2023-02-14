#!/usr/bin/env python3
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import numpy

# 创建一个命令行参数解析器对象
# Create an argument parser object
parser = argparse.ArgumentParser(description='Plot rttest output')

# 向解析器添加一个文件名参数
# Add a filename argument to the parser
parser.add_argument('filename', metavar='filename')

# 解析命令行参数
# Parse command line arguments
args = parser.parse_args()

# 获取文件名
# Get the filename
filename = args.filename

# 定义输出文件名
# Define the output file name
outfile = filename + '_plot'

# 初始化原始行列表
# Initialize raw lines list
rawlines = []

# 以只读方式打开文件
# Open the file in read mode
with open(filename) as f:
    # 读取文件中的所有行
    # Read all the lines in the file
    rawlines = f.readlines()

# 对每一行去除右边的空白并分割成列表
# Strip the right whitespace and split each line into a list
rawlines = [line.rstrip().split(' ') for line in rawlines]

# 将原始行列表转换为 NumPy 数组
# Convert the raw lines list to a NumPy array
array = numpy.array(rawlines)

# 计算延迟（取绝对值）
# Calculate latency (take absolute value)
latency = numpy.absolute(array[1:, 2].astype(int))

# 计算最小延迟
# Calculate minimum latency
min_latency = numpy.min(latency)

# 计算最大延迟
# Calculate maximum latency
max_latency = numpy.max(latency)

# 计算平均延迟
# Calculate mean latency
mean_latency = numpy.mean(latency)

# 输出结果
# Print the results
print('Min latency:', min_latency)
print('Max latency:', max_latency)
print('Mean latency:', mean_latency)

# 计算大于 0.03 ms（30000 ns）的样本数量
# Calculate the number of samples that were above 0.03 ms (30000 ns)
indices = numpy.where(latency > 30000)

# 输出超过指定延迟的样本数量
# Print the number of samples with latency overrun
print('# of samples overrun:', len(indices))
