execute_process(COMMAND "/home/student/Bureau/door-watcher/door_watcher_ws/build/dynamixel_motor-master/dynamixel_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/student/Bureau/door-watcher/door_watcher_ws/build/dynamixel_motor-master/dynamixel_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
