#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>

#include "calculator/calculator.hpp"


Calculator::Calculator(const rclcpp::NodeOptions & node_options)
: Node("calculator", node_options),
  argument_a_(0.0),
  argument_b_(0.0),
  argument_operator_(0),
  argument_result_(0.0),
  argument_formula_("")
{
  RCLCPP_INFO(this->get_logger(), "Run calculator");

  operator_.reserve(4);
  operator_.push_back("+");
  operator_.push_back("-");
  operator_.push_back("*");
  operator_.push_back("/");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  /*subscriber~*/
  const auto QOS_RKL10V =//QoS설정
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  //subscriber 초기화
  arithmetic_argument_subscriber_ = this->create_subscription<ArithmeticArgument>(
    "arithmetic_argument", //인자 1: subscribe할 토픽명
    QOS_RKL10V, //인자 2: QoS설정
    //인자 3: 콜백함수(람다 표현식 사용)
    [this](const ArithmeticArgument::SharedPtr msg) -> void
    {
      argument_a_ = msg->argument_a;
      argument_b_ = msg->argument_b;

      RCLCPP_INFO(//수신받은 시간 출력
        this->get_logger(),
        "Timestamp of the message: sec %ld nanosec %ld",
        msg->stamp.sec,
        msg->stamp.nanosec);

      //받은 데이터 출력
      RCLCPP_INFO(this->get_logger(), "Subscribed argument a: %.2f", argument_a_);
      RCLCPP_INFO(this->get_logger(), "Subscribed argument b: %.2f", argument_b_);
    }
  );
  /*~subscriber*/

/*server~*/
  //콜백함수
  auto get_arithmetic_operator =//람다 표현식 사용
    [this](
    const std::shared_ptr<ArithmeticOperator::Request> request,
    std::shared_ptr<ArithmeticOperator::Response> response) -> void
    {
      //client에게 받은 연산자 저장
      argument_operator_ = request->arithmetic_operator;
      argument_result_ =//계산 후 결과값 리턴받아 저장
        this->calculate_given_formula(argument_a_, argument_b_, argument_operator_);
      response->arithmetic_result = argument_result_;//결과값을 response에 넣어 client가 받아볼 수 있게 한다

      //연산식을 문자열 형태로 저장
      std::ostringstream oss;
      oss << std::to_string(argument_a_) << ' ' <<
        operator_[argument_operator_ - 1] << ' ' <<
        std::to_string(argument_b_) << " = " <<
        argument_result_ << std::endl;
      argument_formula_ = oss.str();

      RCLCPP_INFO(this->get_logger(), "%s", argument_formula_.c_str());
    };

  arithmetic_argument_server_ =
  //create_service함수로 서비스명과 함수를 인자로 받아 server 실체화
    create_service<ArithmeticOperator>("arithmetic_operator", get_arithmetic_operator);


  using namespace std::placeholders;
  arithmetic_action_server_ = rclcpp_action::create_server<ArithmeticChecker>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "arithmetic_checker",
    std::bind(&Calculator::handle_goal, this, _1, _2),
    std::bind(&Calculator::handle_cancel, this, _1),
    std::bind(&Calculator::execute_checker, this, _1)
  );
}

Calculator::~Calculator()
{
}
//계산 함수
float Calculator::calculate_given_formula(
  const float & a,
  const float & b,
  const int8_t & operators)
{
  float argument_result = 0.0;
  ArithmeticOperator::Request arithmetic_operator;

  if (operators == arithmetic_operator.PLUS) {
    argument_result = a + b;
  } else if (operators == arithmetic_operator.MINUS) {
    argument_result = a - b;
  } else if (operators == arithmetic_operator.MULTIPLY) {
    argument_result = a * b;
  } else if (operators == arithmetic_operator.DIVISION) {
    argument_result = a / b;
    if (b == 0.0) {
      RCLCPP_ERROR(this->get_logger(), "ZeroDivisionError!");
      argument_result = 0.0;
      return argument_result;
    }
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Please make sure arithmetic operator(plus, minus, multiply, division).");
    argument_result = 0.0;
  }

  return argument_result;
}
/*~server*/

rclcpp_action::GoalResponse Calculator::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ArithmeticChecker::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Calculator::handle_cancel(
  const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Calculator::execute_checker(const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Execute arithmetic_checker action!");
  rclcpp::Rate loop_rate(1);

  auto feedback_msg = std::make_shared<ArithmeticChecker::Feedback>();
  float total_sum = 0.0;
  float goal_sum = goal_handle->get_goal()->goal_sum;

  while ((total_sum < goal_sum) && rclcpp::ok()) {
    total_sum += argument_result_;
    feedback_msg->formula.push_back(argument_formula_);
    if (argument_formula_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Please check your formula");
      break;
    }
    RCLCPP_INFO(this->get_logger(), "Feedback: ");
    for (const auto & formula : feedback_msg->formula) {
      RCLCPP_INFO(this->get_logger(), "\t%s", formula.c_str());
    }
    goal_handle->publish_feedback(feedback_msg);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    auto result = std::make_shared<ArithmeticChecker::Result>();
    result->all_formula = feedback_msg->formula;
    result->total_sum = total_sum;
    goal_handle->succeed(result);
  }
}
