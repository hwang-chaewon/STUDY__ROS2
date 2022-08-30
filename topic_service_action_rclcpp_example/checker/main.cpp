#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h" //main의 인자를 쉽게 확인할 수 있는 라이브러리

#include "checker/checker.hpp"


void print_help()
{
  printf("For Node node:\n");
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

  //main에 들어온 argc, argv를 rclcpp::init에 넘겨줘, rclcpp가 '--ros-args'인자를 확인하게 하는 과정
  rclcpp::init(argc, argv);

  float goal_total_sum = 50.0;
  //rcutils_cli_get_option함수: 실행 인자를 확인해 문자열 포인터로 변환
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-g");
  if (nullptr != cli_option) {
    //실행 인자 문자열 포인터를 원하는 변수 타입으로 변경
    goal_total_sum = std::stof(cli_option);
  }
  printf("goal_total_sum : %2.f\n", goal_total_sum);

  //노드의 생성 인자로 넘겨주기
  auto checker = std::make_shared<Checker>(goal_total_sum);

  rclcpp::spin(checker);

  rclcpp::shutdown();

  return 0;
}
