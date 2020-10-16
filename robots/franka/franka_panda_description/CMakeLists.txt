cmake_minimum_required(VERSION 2.8.3)

project(franka_panda_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir meshes robots)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
