#include <fcntl.h>
#include <getopt.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "/opt/ros/foxy/include/rclcpp/rclcpp.hpp"
#include "/opt/ros/foxy/include/rcutils/cmdline_parser.h"

#include "arithmetic/operator.hpp"

using namespace std::chrono_literals;

Operator::Operator(const rclcpp::NodeOptions & node_options)
: Node("operator", node_options)
{
  //서비스명(arithmetic_operator)를 인자로 받아 client 실체화
  arithmetic_service_client_ = this->create_client<ArithmeticOperator>("arithmetic_operator");
  while (!arithmetic_service_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }
}

Operator::~Operator()
{
}

//service client로서 요청을 보내는 함수
void Operator::send_request()
{
  // +,-,*,/에 대응하는 1,2,3,4 중 랜덤 숫자 생성
  auto request = std::make_shared<ArithmeticOperator::Request>();//msg_srv_action_interface_example의 ArithmeticOperator_Request_구조체에서 정의했던 Request
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> distribution(1, 4);
  request->arithmetic_operator = distribution(gen);

  using ServiceResponseFuture = rclcpp::Client<ArithmeticOperator>::SharedFuture;

  //콜백함수(람다 표현식)
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      //response 출력
      RCLCPP_INFO(this->get_logger(), "Result %.2f", response->arithmetic_result);
      return;
    };

  auto future_result =
  //비동기식(async)으로 서비스 요청
    arithmetic_service_client_->async_send_request(request, response_received_callback);
}

int getch()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

bool pull_trigger()
{
  const uint8_t KEY_ENTER = 10;
  while (1) {
    if (kbhit()) {
      uint8_t c = getch();
      if (c == KEY_ENTER) {
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}

void print_help()
{
  printf("For operator node:\n");
  printf("node_name [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}

int main(int argc, char * argv[])
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);

  //instance만들기
  auto operator_node = std::make_shared<Operator>();

  //send_request함수 호출
  while (rclcpp::ok()) {
    rclcpp::spin_some(operator_node);
    operator_node->send_request();

    printf("Press Enter for next service call.\n");
    if (pull_trigger() == false) {
      rclcpp::shutdown();
      return 0;
    }
  }
}
