execute_process(COMMAND "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/build/crbot-flexbe_behaviors/crbot-flexbe_flexbe_states/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/build/crbot-flexbe_behaviors/crbot-flexbe_flexbe_states/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
