cmake_minimum_required(VERSION 2.8.3)
project(waypoint_planning)

find_package(catkin REQUIRED COMPONENTS
        rospy
        rqt_gui
        rqt_gui_py
        )


# To enable assertions when compiled in release mode.
catkin_package()
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS scripts/waypoint_planning
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(DIRECTORY launch resource
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
