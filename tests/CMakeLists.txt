cmake_minimum_required(VERSION 3.22)
project(chassis_mission_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()

add_executable(mission_test mission_test.cpp ../applications/mission/mission.cpp)
target_include_directories(mission_test PRIVATE ../applications)

add_executable(chassis_control_test chassis_control_test.cpp
  ../applications/mission/mission.cpp ../applications/can_task.cpp
  ../sp_middleware/motor/rm_motor/rm_motor.cpp ../sp_middleware/tools/pid/pid.cpp
  ../sp_middleware/tools/math_tools/math_tools.cpp ../sp_middleware/tools/mecanum/mecanum.cpp)
target_include_directories(chassis_control_test PRIVATE fakes ../applications ../sp_middleware)
target_compile_definitions(chassis_control_test PRIVATE M_PI=3.14159265358979323846)
if(MSVC)
  target_compile_options(chassis_control_test PRIVATE /FIcstddef)
else()
  target_compile_options(chassis_control_test PRIVATE -include cstddef -Wall -Wextra -Werror)
  target_compile_options(mission_test PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME mission_behavior COMMAND mission_test)
add_test(NAME chassis_control_behavior COMMAND chassis_control_test)
set_tests_properties(mission_behavior chassis_control_behavior PROPERTIES TIMEOUT 30)
