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

// C++ wrapper for Miguel Masmano Tello's implementation of the TLSF memory allocator
// Implements the allocator_traits template

#ifndef TLSF_CPP__TLSF_HPP_
#define TLSF_CPP__TLSF_HPP_

#include <cstring>
#include <iostream>
#include <new>
#include <stdexcept>

#include "tlsf/tlsf.h"

/**
 * @brief TLSF 堆分配器 (TLSF heap allocator)
 *
 * @tparam T 分配器的值类型 (Value type for the allocator)
 * @tparam DefaultPoolSize 默认内存池大小 (Default memory pool size)
 */
template <typename T, size_t DefaultPoolSize = 1024 * 1024> struct tlsf_heap_allocator
{
  // Needed for std::allocator_traits
  using value_type = T;

  /**
   * @brief 构造函数，指定内存池大小 (Constructor with specified memory pool size)
   *
   * @param size 内存池大小 (Memory pool size)
   */
  explicit tlsf_heap_allocator(size_t size) : memory_pool(nullptr), pool_size(size)
  {
    initialize(size);
  }

  // Needed for std::allocator_traits
  /**
   * @brief 默认构造函数，使用默认内存池大小 (Default constructor with default memory pool size)
   */
  tlsf_heap_allocator() : memory_pool(nullptr) { initialize(DefaultPoolSize); }

  // Needed for std::allocator_traits
  /**
   * @brief 拷贝构造函数 (Copy constructor)
   *
   * @tparam U 分配器的值类型 (Value type for the allocator)
   * @param alloc 被拷贝的分配器 (The allocator to be copied)
   */
  template <typename U>
  tlsf_heap_allocator(const tlsf_heap_allocator<U> & alloc)
  : memory_pool(alloc.memory_pool), pool_size(alloc.pool_size)
  {
  }

  /**
   * @brief 拷贝构造函数，支持不同默认内存池大小 (Copy constructor with support for different default memory pool sizes)
   *
   * @tparam U 分配器的值类型 (Value type for the allocator)
   * @tparam OtherDefaultSize 另一个分配器的默认内存池大小 (Default memory pool size for another allocator)
   * @param alloc 被拷贝的分配器 (The allocator to be copied)
   */
  template <typename U, size_t OtherDefaultSize>
  tlsf_heap_allocator(const tlsf_heap_allocator<U> & alloc)
  : memory_pool(alloc.memory_pool), pool_size(alloc.pool_size)
  {
  }

  /**
   * @brief 初始化内存池 (Initialize the memory pool)
   *
   * @param size 内存池大小 (Memory pool size)
   * @return 初始化后的内存池大小 (Initialized memory pool size)
   */
  size_t initialize(size_t size)
  {
    pool_size = size;
    if (!memory_pool) {
      memory_pool = new char[pool_size];
      memset(memory_pool, 0, pool_size);
      init_memory_pool(pool_size, memory_pool);
    }
    return pool_size;
  }

  /**
   * @brief 析构函数，销毁内存池 (Destructor, destroys the memory pool)
   */
  ~tlsf_heap_allocator()
  {
    if (memory_pool) {
      destroy_memory_pool(memory_pool);
      memory_pool = nullptr;
    }
  }

  // Needed for std::allocator_traits
  /**
   * @brief 分配内存 (Allocate memory)
   *
   * @param size 请求分配的大小 (Requested allocation size)
   * @return 指向分配内存的指针 (Pointer to the allocated memory)
   */
  T * allocate(size_t size)
  {
    T * ptr = static_cast<T *>(tlsf_malloc(size * sizeof(T)));
    if (ptr == NULL && size > 0) {
      throw std::bad_alloc();
    }
    return ptr;
  }

  // Needed for std::allocator_traits
  /**
   * @brief 释放内存 (Deallocate memory)
   *
   * @param ptr 指向要释放内存的指针 (Pointer to the memory to be deallocated)
   * @param size 未使用，兼容 std::allocator_traits (Unused, for compatibility with std::allocator_traits)
   */
  void deallocate(T * ptr, size_t) { tlsf_free(ptr); }

  /**
   * @brief 用于重新绑定分配器类型 (Used for rebinding allocator types)
   *
   * @tparam U 分配器的值类型 (Value type for the allocator)
   */
  template <typename U> struct rebind
  {
    typedef tlsf_heap_allocator<U> other;
  };

  char * memory_pool; ///< 内存池指针 (Memory pool pointer)
  size_t pool_size;   ///< 内存池大小 (Memory pool size)
};

/**
 * @brief 比较两个 tlsf_heap_allocator 对象是否相等 (Compare two tlsf_heap_allocator objects for equality)
 * @tparam T 第一个 tlsf_heap_allocator 的类型参数 (Type parameter of the first tlsf_heap_allocator)
 * @tparam U 第二个 tlsf_heap_allocator 的类型参数 (Type parameter of the second tlsf_heap_allocator)
 * @param a 第一个 tlsf_heap_allocator 对象 (The first tlsf_heap_allocator object)
 * @param b 第二个 tlsf_heap_allocator 对象 (The second tlsf_heap_allocator object)
 * @return 如果两个 tlsf_heap_allocator 对象的内存池相同，则返回 true，否则返回 false (Returns true if the memory pools of both tlsf_heap_allocator objects are the same, otherwise returns false)
 */
template <typename T, typename U>
constexpr bool
operator==(const tlsf_heap_allocator<T> & a, const tlsf_heap_allocator<U> & b) noexcept
{
  return a.memory_pool ==
         b.memory_pool; // 比较两个 tlsf_heap_allocator 对象的内存池是否相同 (Compare if the memory pools of the two tlsf_heap_allocator objects are the same)
}

/**
 * @brief 比较两个 tlsf_heap_allocator 对象是否不相等 (Compare two tlsf_heap_allocator objects for inequality)
 * @tparam T 第一个 tlsf_heap_allocator 的类型参数 (Type parameter of the first tlsf_heap_allocator)
 * @tparam U 第二个 tlsf_heap_allocator 的类型参数 (Type parameter of the second tlsf_heap_allocator)
 * @param a 第一个 tlsf_heap_allocator 对象 (The first tlsf_heap_allocator object)
 * @param b 第二个 tlsf_heap_allocator 对象 (The second tlsf_heap_allocator object)
 * @return 如果两个 tlsf_heap_allocator 对象的内存池不相同，则返回 true，否则返回 false (Returns true if the memory pools of both tlsf_heap_allocator objects are not the same, otherwise returns false)
 */
template <typename T, typename U>
constexpr bool
operator!=(const tlsf_heap_allocator<T> & a, const tlsf_heap_allocator<U> & b) noexcept
{
  return a.memory_pool !=
         b.memory_pool; // 比较两个 tlsf_heap_allocator 对象的内存池是否不相同 (Compare if the memory pools of the two tlsf_heap_allocator objects are not the same)
}

/**
 * @brief 比较两个带有大小参数的 tlsf_heap_allocator 对象是否相等 (Compare two tlsf_heap_allocator objects with size parameters for equality)
 * @tparam T 第一个 tlsf_heap_allocator 的类型参数 (Type parameter of the first tlsf_heap_allocator)
 * @tparam U 第二个 tlsf_heap_allocator 的类型参数 (Type parameter of the second tlsf_heap_allocator)
 * @tparam X 第一个 tlsf_heap_allocator 的大小参数 (Size parameter of the first tlsf_heap_allocator)
 * @tparam Y 第二个 tlsf_heap_allocator 的大小参数 (Size parameter of the second tlsf_heap_allocator)
 * @param a 第一个 tlsf_heap_allocator 对象 (The first tlsf_heap_allocator object)
 * @param b 第二个 tlsf_heap_allocator 对象 (The second tlsf_heap_allocator object)
 * @return 如果两个 tlsf_heap_allocator 对象的内存池相同，则返回 true，否则返回 false (Returns true if the memory pools of both tlsf_heap_allocator objects are the same, otherwise returns false)
 */
template <typename T, typename U, size_t X, size_t Y>
constexpr bool
operator==(const tlsf_heap_allocator<T, X> & a, const tlsf_heap_allocator<U, Y> & b) noexcept
{
  return a.memory_pool ==
         b.memory_pool; // 比较两个带有大小参数的 tlsf_heap_allocator 对象的内存池是否相同 (Compare if the memory pools of the two tlsf_heap_allocator objects with size parameters are the same)
}

/**
 * @brief 比较两个带有大小参数的 tlsf_heap_allocator 对象是否不相等 (Compare two tlsf_heap_allocator objects with size parameters for inequality)
 * @tparam T 第一个 tlsf_heap_allocator 的类型参数 (Type parameter of the first tlsf_heap_allocator)
 * @tparam U 第二个 tlsf_heap_allocator 的类型参数 (Type parameter of the second tlsf_heap_allocator)
 * @tparam X 第一个 tlsf_heap_allocator 的大小参数 (Size parameter of the first tlsf_heap_allocator)
 * @tparam Y 第二个 tlsf_heap_allocator 的大小参数 (Size parameter of the second tlsf_heap_allocator)
 * @param a 第一个 tlsf_heap_allocator 对象 (The first tlsf_heap_allocator object)
 * @param b 第二个 tlsf_heap_allocator 对象 (The second tlsf_heap_allocator object)
 * @return 如果两个 tlsf_heap_allocator 对象的内存池不相同，则返回 true，否则返回 false (Returns true if the memory pools of both tlsf_heap_allocator objects are not the same, otherwise returns false)
 */
template <typename T, typename U, size_t X, size_t Y>
constexpr bool
operator!=(const tlsf_heap_allocator<T, X> & a, const tlsf_heap_allocator<U, Y> & b) noexcept
{
  return a.memory_pool !=
         b.memory_pool; // 比较两个带有大小参数的 tlsf_heap_allocator 对象的内存池是否不相同 (Compare if the memory pools of the two tlsf_heap_allocator objects with size parameters are not the same)
}

#endif // TLSF_CPP__TLSF_HPP_
