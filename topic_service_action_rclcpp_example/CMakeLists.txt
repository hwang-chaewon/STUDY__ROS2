################################################################################
# Cmake설정: Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(topic_service_action_rclcpp_example)

#cmake는 별다른 설정이 없으면 C99와 C++14를 기본으로 사용
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#컴파일러는 기본으로 GNU사용, Clang 사용 가능
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(msg_srv_action_interface_example REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

#Build
#argument노드 실행을 위해 필요한 의존성 패키지들
add_executable(argument src/arithmetic/argument.cpp)
ament_target_dependencies(argument
  msg_srv_action_interface_example
  rclcpp
)

#calculator노드 실행을 위해 필요한 의존성 패키지들
add_executable(calculator src/calculator/main.cpp src/calculator/calculator.cpp)
ament_target_dependencies(calculator
  msg_srv_action_interface_example
  rclcpp
  rclcpp_action
)

#checker노드 실행을 위해 필요한 의존성 패키지들
add_executable(checker src/checker/main.cpp src/checker/checker.cpp)
ament_target_dependencies(checker
  msg_srv_action_interface_example
  rclcpp
  rclcpp_action
)

#operator노드 실행을 위해 필요한 의존성 패키지들
add_executable(operator src/arithmetic/operator.cpp)
ament_target_dependencies(operator
  msg_srv_action_interface_example
  rclcpp
)

#Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

#Install
install(TARGETS
  argument
  calculator
  checker
  operator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Find and load build settings from external packages
################################################################################
#Macro for ament package
ament_package() #필수
