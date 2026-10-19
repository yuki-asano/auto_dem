cmake_minimum_required(VERSION 2.8.0)
project(auto_dem)

find_package(catkin REQUIRED COMPONENTS
  aet_instrument
  roseus
  rospy
  std_msgs
  xacro
)

#catkin_python_setup()

catkin_package()

###
### fr3-dielec.l generation
###
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/fr3-dielec.l
  COMMAND rosrun euscollada collada2eus -I fr3_dielec.urdf -C ${PROJECT_SOURCE_DIR}/models/fr3_dielec.yaml -O fr3-dielec.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/fr3_dielec.urdf ${PROJECT_SOURCE_DIR}/models/fr3_dielec.yaml)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/fr3_dielec.urdf
  COMMAND rosrun xacro xacro --inorder fr3_dielec.urdf.xacro > fr3_dielec.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/fr3_dielec.urdf.xacro)

add_custom_target(generate_fr3-dielec_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/fr3-dielec.l)

# dielectrometer
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/dielectrometer.urdf
  COMMAND rosrun xacro xacro --inorder ${PROJECT_SOURCE_DIR}/models/dielectrometer.urdf.xacro > dielectrometer.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/dielectrometer.urdf.xacro)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/dielectrometer.l
  COMMAND rosrun euscollada collada2eus -I dielectrometer.urdf -O dielectrometer.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/dielectrometer.urdf)

add_custom_target(generate_dielectrometer_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/dielectrometer.urdf ${PROJECT_SOURCE_DIR}/models/dielectrometer.l)


# dielectrometer_workstand
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/dielectrometer_workstand.urdf
  COMMAND rosrun xacro xacro --inorder ${PROJECT_SOURCE_DIR}/models/dielectrometer_workstand.urdf.xacro > dielectrometer_workstand.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/dielectrometer_workstand.urdf.xacro)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/dielectrometer_workstand.l
  COMMAND rosrun euscollada collada2eus -I dielectrometer_workstand.urdf -O dielectrometer_workstand.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/dielectrometer_workstand.urdf)

add_custom_target(generate_dielectrometer_workstand_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/dielectrometer_workstand.urdf ${PROJECT_SOURCE_DIR}/models/dielectrometer_workstand.l)


# dielectrometer_table
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/dielectrometer_table.urdf
  COMMAND rosrun xacro xacro --inorder ${PROJECT_SOURCE_DIR}/models/dielectrometer_table.urdf.xacro > dielectrometer_table.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/dielectrometer_table.urdf.xacro)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/dielectrometer_table.l
  COMMAND rosrun euscollada collada2eus -I dielectrometer_table.urdf -O dielectrometer_table.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/dielectrometer_table.urdf)

add_custom_target(generate_dielectrometer_table_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/dielectrometer_table.urdf ${PROJECT_SOURCE_DIR}/models/dielectrometer_table.l)


# dielectrometer exp
# this model includes all geometry data of dielectrometer experiments like machines and tools
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/auto_dem.urdf
  COMMAND rosrun xacro xacro --inorder auto_dem.urdf.xacro > auto_dem.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/auto_dem.urdf.xacro)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/auto_dem.l
  COMMAND rosrun euscollada collada2eus -I auto_dem.urdf -O auto_dem.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/models
  DEPENDS ${PROJECT_SOURCE_DIR}/models/auto_dem.urdf)

add_custom_target(generate_auto_dem_urdf ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/auto_dem.urdf ${PROJECT_SOURCE_DIR}/models/auto_dem.l)


#############
## Install ##
#############
foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
