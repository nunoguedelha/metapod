## Copyright (c) 2012, 2013 Aldebaran Robotics. All rights reserved
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING.bsd file
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/robotbuilder/include
    ${PROJECT_BINARY_DIR}/robotbuilder/include
    ${Boost_INCLUDE_DIRS}) # Eigen is already in the include path

IF(${MODEL_JOINT_FWD_DYN_FROM_URDF})
  SET(MODEL_JOINT_FWD_DYN "from_urdf")
ELSEIF(!${MODEL_JOINT_FWD_DYN_FROM_URDF} & ${MODEL_JOINT_FWD_DYN_LOCAL})
  SET(MODEL_JOINT_FWD_DYN "all_on")
ELSEIF(!${MODEL_JOINT_FWD_DYN_FROM_URDF} & !${MODEL_JOINT_FWD_DYN_LOCAL})
  SET(MODEL_JOINT_FWD_DYN "all_off")
ENDIF()

SET(MODEL_JOINT_FWD_DYN "from_urdf")
ADD_DEFINITIONS(" -DJOINT_FWD_DYN=${MODEL_JOINT_FWD_DYN}")

ADD_EXECUTABLE(metapodfromurdf
  src/metapodfromurdf.cpp)
TARGET_LINK_LIBRARIES(metapodfromurdf
  ${Boost_LIBRARIES}
  metapod_robotbuilder)
IF(USE_URDF_FROM_ROS_FUERTE)
  SET_SOURCE_FILES_PROPERTIES("src/metapodfromurdf.cpp" PROPERTIES
      COMPILE_FLAGS "-DUSE_URDF_FROM_ROS_FUERTE")
  FOREACH(dep rosconsole urdf)
    ADD_ROSPACK_DEPENDENCY(${dep})
    ROSPACK_USE_DEPENDENCY(metapodfromurdf ${dep})
  ENDFOREACH()
ELSE()
  TARGET_LINK_LIBRARIES(metapodfromurdf ${urdfdom_LIBRARIES})
ENDIF()
SET(METAPODFROMURDF_EXECUTABLE metapodfromurdf CACHE
  STRING "Set by cmake if metapodfromurdf is built, by the user otherwise")

INSTALL(PROGRAMS ${PROJECT_BINARY_DIR}/metapodfromurdf/metapodfromurdf
  DESTINATION bin)
