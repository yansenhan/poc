# Install script for directory: /home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lijixiang/Thu_unitree/Unitree_nav/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slamware_ros_sdk/msg" TYPE FILE FILES
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalBool.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalInt8.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalInt16.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalInt32.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalInt64.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalUInt8.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalUInt16.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalUInt32.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalUInt64.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalFlt32.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalFlt64.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/Vec2DInt32.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/Vec2DFlt32.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/Line2DFlt32.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/Line2DFlt32Array.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/RectInt32.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/RectFlt32.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/RobotDeviceInfo.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/MapKind.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/ArtifactUsage.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/SensorType.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/ImpactType.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/BasicSensorInfo.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/BasicSensorInfoArray.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/BasicSensorValue.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/BasicSensorValueData.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/BasicSensorValueDataArray.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/ActionDirection.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/RobotBasicState.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/SyncMapRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/MoveOptionFlag.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/MoveOptions.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/MoveByDirectionRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/MoveByThetaRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/MoveToRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/MoveToLocationsRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/RotateToRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/RotateRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/LocalizationMovement.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/OptionalLocalizationMovement.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/LocalizationOptions.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/RecoverLocalizationRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/ClearMapRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/SetMapUpdateRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/SetMapLocalizationRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/GoHomeRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/CancelActionRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/AddLineRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/AddLinesRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/RemoveLineRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/ClearLinesRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/MoveLineRequest.msg"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/msg/MoveLinesRequest.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slamware_ros_sdk/srv" TYPE FILE FILES
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/srv/SyncGetStcm.srv"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/srv/SyncSetStcm.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slamware_ros_sdk/cmake" TYPE FILE FILES "/home/lijixiang/Thu_unitree/Unitree_nav/build/slamware_ros_sdk/catkin_generated/installspace/slamware_ros_sdk-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lijixiang/Thu_unitree/Unitree_nav/devel/include/slamware_ros_sdk")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/lijixiang/Thu_unitree/Unitree_nav/devel/share/roseus/ros/slamware_ros_sdk")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lijixiang/Thu_unitree/Unitree_nav/devel/share/common-lisp/ros/slamware_ros_sdk")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/lijixiang/Thu_unitree/Unitree_nav/devel/share/gennodejs/ros/slamware_ros_sdk")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/lijixiang/miniconda3/bin/python3" -m compileall "/home/lijixiang/Thu_unitree/Unitree_nav/devel/lib/python3/dist-packages/slamware_ros_sdk")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/lijixiang/Thu_unitree/Unitree_nav/devel/lib/python3/dist-packages/slamware_ros_sdk")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lijixiang/Thu_unitree/Unitree_nav/build/slamware_ros_sdk/catkin_generated/installspace/slamware_ros_sdk.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slamware_ros_sdk/cmake" TYPE FILE FILES "/home/lijixiang/Thu_unitree/Unitree_nav/build/slamware_ros_sdk/catkin_generated/installspace/slamware_ros_sdk-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slamware_ros_sdk/cmake" TYPE FILE FILES
    "/home/lijixiang/Thu_unitree/Unitree_nav/build/slamware_ros_sdk/catkin_generated/installspace/slamware_ros_sdkConfig.cmake"
    "/home/lijixiang/Thu_unitree/Unitree_nav/build/slamware_ros_sdk/catkin_generated/installspace/slamware_ros_sdkConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slamware_ros_sdk" TYPE FILE FILES "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lijixiang/Thu_unitree/Unitree_nav/devel/lib/libslamware_ros_sdk.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/slamware_ros_sdk/slamware_ros_sdk_server_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/slamware_ros_sdk/slamware_ros_sdk_server_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/slamware_ros_sdk/slamware_ros_sdk_server_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/slamware_ros_sdk" TYPE EXECUTABLE FILES "/home/lijixiang/Thu_unitree/Unitree_nav/devel/lib/slamware_ros_sdk/slamware_ros_sdk_server_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/slamware_ros_sdk/slamware_ros_sdk_server_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/slamware_ros_sdk/slamware_ros_sdk_server_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/slamware_ros_sdk/slamware_ros_sdk_server_node"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/slamware_ros_sdk/slamware_ros_sdk_server_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/slamware_ros_sdk" TYPE DIRECTORY FILES "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/include/slamware_ros_sdk/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/\\.svn$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slamware_ros_sdk/launch" TYPE FILE FILES
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/launch/slamware_ros_sdk_server_node.launch"
    "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/launch/view_slamware_ros_sdk_server_node.launch"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slamware_ros_sdk" TYPE DIRECTORY FILES "/home/lijixiang/Thu_unitree/Unitree_nav/src/slamware_ros_sdk/rviz")
endif()

