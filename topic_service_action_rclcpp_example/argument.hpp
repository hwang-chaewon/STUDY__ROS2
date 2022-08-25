#ifndef ARITHMETIC__ARGUMENT_HPP_
#define ARITHMETIC__ARGUMENT_HPP_

#include <chrono>//시간 라이브러리
#include <memory>//동적 메모리 라이브러리
#include <string>//문자열 라이브러리
#include <utility>//서로 다른 도메인을 사용할 수 있게 해주는 라이브러리

#include "rclcpp/rclcpp.hpp"

#include "msg_srv_action_interface_example/msg/arithmetic_argument.hpp"//인터페이스를 담고 있는 헤더파일


class Argument : public rclcpp::Node
{
public:
  using ArithmeticArgument = msg_srv_action_interface_example::msg::ArithmeticArgument;

  //노드 생성시 다양한 옵션을 정하기 위한 NodeOptions관련 설정을 하는 생성자
  explicit Argument(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Argument();

private:
  void publish_random_arithmetic_arguments();
  void update_parameter();

  float min_random_num_;
  float max_random_num_;

  //스레드를 이용한 지속적 실행을 위해 그냥 Ptr이 아닌 SharedPtr사용
  rclcpp::Publisher<ArithmeticArgument>::SharedPtr arithmetic_argument_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};
#endif  // ARITHMETIC__ARGUMENT_HPP_
