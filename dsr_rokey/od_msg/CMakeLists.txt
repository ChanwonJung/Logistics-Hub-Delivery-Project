cmake_minimum_required(VERSION 3.8)
project(od_msg)

# ament 및 rosidl 설정
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 서비스 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SrvDepthPosition.srv"
)

# 런타임 의존성 내보내기
ament_export_dependencies(rosidl_default_runtime)

ament_package()
