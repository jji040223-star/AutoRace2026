cmake_minimum_required(VERSION 3.8)
project(autorace_track NONE)
find_package(ament_cmake REQUIRED)
# 생성물(USD, 분류 JSON)과 빌드 설정만 설치. 도구(tools/)는 소스 트리에서 venv 로 실행.
install(DIRECTORY usd config DESTINATION share/${PROJECT_NAME})
install(FILES data/generated/track_elements.json DESTINATION share/${PROJECT_NAME}/data)
ament_package()
