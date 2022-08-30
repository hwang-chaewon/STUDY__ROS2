import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#launch파일은 기본적으로 generate_launch_description()를 이용한다
def generate_launch_description():
    param_dir = LaunchConfiguration(#생성자
        'param_dir', #파라미터 디렉토리로 param_dir을 설정
        default=os.path.join(
            get_package_share_directory('topic_service_action_rclcpp_example'),
            'param', #파라미터 설정 파일: topic_service_action_rclcpp_example/param/arithmetic_config.yaml
            'arithmetic_config.yaml'))


    return LaunchDescription([
        DeclareLaunchArgument(#param_dir을 launch인수로 선언
            'param_dir',
            default_value=param_dir,
            description='Full path of parameter file'),

        #Node(): 실행할 노드 설정
        Node(
            package='topic_service_action_rclcpp_example',
            executable='argument', #executable: 실행할 노드 이름
            name='argument', #노드 실행할 때 실제로 사용할 이름. 보통 원래 이름대로 적음
            parameters=[param_dir], #사용할 파라미터(이 경우 arithmetic_config.yaml 파일)
            output='screen'#output: Logging설정. screen의 경우 터미널창에 표시하겠다는 것
            #remappings=[('/arithmetic_argument','/argument')]    #remappings: 토픽, 서비스, 액션의 이름 변경 가능. 여기서는 /arithmetic_argument라는 토픽을 /argument로 이름을 바꿈
            ),

        Node(
            package='topic_service_action_rclcpp_example',
            executable='calculator',
            name='calculator',
            parameters=[param_dir],
            output='screen',),
    ])
