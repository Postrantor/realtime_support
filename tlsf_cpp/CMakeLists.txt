cmake_minimum_required(VERSION 3.5)

# 项目名称：tlsf_cpp Project name: tlsf_cpp
project(tlsf_cpp)

# 如果当前系统是 Windows、OSX 或者 Android If the current system is Windows, OSX, or
# Android
if(WIN32
   OR APPLE
   OR ANDROID)
  # 输出一条状态消息，说明 tlsf 分配器不支持这些平台，跳过编译 Output a status message, stating that the
  # tlsf allocator does not target these platforms, skipping compilation
  message(
    STATUS "tlsf allocator does not target Windows, OSX or Android, skipping..."
  )
  # 返回，不再执行后续的 CMake 指令 Return, no further CMake commands will be executed
  return()
endif()

# 默认使用 C++17 标准 (Default to C++17 standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # 设置 C++ 标准为 17 (Set C++ standard to 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON) # 要求必须使用该标准 (Require the use of this
                                      # standard)
endif()

# 当编译器为 GNUCXX 或 Clang 时，添加编译选项 -Wall 和 -Wextra (Add compile options -Wall and
# -Wextra when the compiler is GNUCXX or Clang)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# 寻找依赖的包 (Find required packages)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tlsf REQUIRED)

# 添加 tlsf_cpp 库并设置为 INTERFACE 类型 (Add tlsf_cpp library and set as INTERFACE
# type)
add_library(tlsf_cpp INTERFACE)
# 设置库的头文件目录 (Set the header directories for the library)
target_include_directories(
  tlsf_cpp INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# 链接库到其它库 (Link the library to other libraries)
target_link_libraries(tlsf_cpp INTERFACE tlsf::tlsf)

# 添加可执行文件 tlsf_allocator_example 并设置源文件为 example/allocator_example.cpp (Add
# executable tlsf_allocator_example and set source file as
# example/allocator_example.cpp)
add_executable(tlsf_allocator_example example/allocator_example.cpp)
# 链接可执行文件到其它库 (Link the executable to other libraries)
target_link_libraries(tlsf_allocator_example rclcpp::rclcpp ${std_msgs_TARGETS}
                      tlsf_cpp)

# 安装可执行文件到 lib/${PROJECT_NAME} 目录下 (Install the executable to
# lib/${PROJECT_NAME} directory)
install(TARGETS tlsf_allocator_example DESTINATION lib/${PROJECT_NAME})

# 导出目标 (Export targets)
ament_export_targets(export_tlsf_cpp)

# 导出依赖 (Export dependencies)
ament_export_dependencies("tlsf")

# 如果构建测试 (If building tests)
if(BUILD_TESTING)
  # 查找 ament_lint_auto 包，用于代码风格检查 (Find the ament_lint_auto package for code
  # style checking)
  find_package(ament_lint_auto REQUIRED)
  # 自动查找测试依赖项 (Automatically find test dependencies)
  ament_lint_auto_find_test_dependencies()

  # 获取 rmw 实现的类型支持以包含/链接相应的接口 (Get typesupport of rmw implementation to
  # include/link against the corresponding interfaces)

  # 定义一个宏 add_gtest (Define a macro named add_gtest)
  macro(add_gtest)
    # 添加一个基于 gtest 的测试 (Add a gtest-based test)
    ament_add_gtest(
      test_tlsf${target_suffix} # 测试目标名称 (Test target name)
      "test/test_tlsf.cpp" # 测试源文件 (Test source file)
      TIMEOUT
      15 # 测试超时时间 (Test timeout duration)
      ENV # 环境变量设置 (Environment variable settings)
      RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
      RMW_IMPLEMENTATION=${rmw_implementation})
    # 如果目标 test_tlsf${target_suffix} 存在 (If the target test_tlsf${target_suffix}
    # exists)
    if(TARGET test_tlsf${target_suffix})
      # 为目标添加编译定义 (Add compile definitions for the target)
      target_compile_definitions(
        test_tlsf${target_suffix}
        PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
      # 设置目标链接库 (Set target link libraries)
      target_link_libraries(test_tlsf${target_suffix} rclcpp::rclcpp
                            ${std_msgs_TARGETS} tlsf_cpp)
    endif()
  endmacro()

  # 对每个 rmw 实现调用 add_gtest 宏 (Call the add_gtest macro for each rmw
  # implementation)
  call_for_each_rmw_implementation(add_gtest)
endif()

# 打包 ament 包 (Package the ament package)
ament_package()

# 将 include/ 目录安装到 include/${PROJECT_NAME} (Install the include/ directory to
# include/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# 安装目标 tlsf_cpp (Install the target tlsf_cpp)
install(TARGETS tlsf_cpp EXPORT export_tlsf_cpp)
