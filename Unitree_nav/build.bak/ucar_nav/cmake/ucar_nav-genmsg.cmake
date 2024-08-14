# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ucar_nav: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iucar_nav:/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ucar_nav_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg" NAME_WE)
add_custom_target(_ucar_nav_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucar_nav" "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ucar_nav
  "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_nav
)

### Generating Services

### Generating Module File
_generate_module_cpp(ucar_nav
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_nav
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ucar_nav_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ucar_nav_generate_messages ucar_nav_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg" NAME_WE)
add_dependencies(ucar_nav_generate_messages_cpp _ucar_nav_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_nav_gencpp)
add_dependencies(ucar_nav_gencpp ucar_nav_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_nav_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ucar_nav
  "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_nav
)

### Generating Services

### Generating Module File
_generate_module_eus(ucar_nav
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_nav
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ucar_nav_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ucar_nav_generate_messages ucar_nav_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg" NAME_WE)
add_dependencies(ucar_nav_generate_messages_eus _ucar_nav_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_nav_geneus)
add_dependencies(ucar_nav_geneus ucar_nav_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_nav_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ucar_nav
  "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_nav
)

### Generating Services

### Generating Module File
_generate_module_lisp(ucar_nav
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_nav
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ucar_nav_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ucar_nav_generate_messages ucar_nav_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg" NAME_WE)
add_dependencies(ucar_nav_generate_messages_lisp _ucar_nav_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_nav_genlisp)
add_dependencies(ucar_nav_genlisp ucar_nav_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_nav_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ucar_nav
  "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_nav
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ucar_nav
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_nav
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ucar_nav_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ucar_nav_generate_messages ucar_nav_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg" NAME_WE)
add_dependencies(ucar_nav_generate_messages_nodejs _ucar_nav_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_nav_gennodejs)
add_dependencies(ucar_nav_gennodejs ucar_nav_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_nav_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ucar_nav
  "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_nav
)

### Generating Services

### Generating Module File
_generate_module_py(ucar_nav
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_nav
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ucar_nav_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ucar_nav_generate_messages ucar_nav_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/msg/Boxinfo.msg" NAME_WE)
add_dependencies(ucar_nav_generate_messages_py _ucar_nav_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_nav_genpy)
add_dependencies(ucar_nav_genpy ucar_nav_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_nav_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_nav)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_nav
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ucar_nav_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_nav)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_nav
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ucar_nav_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_nav)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_nav
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ucar_nav_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_nav)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_nav
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ucar_nav_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_nav)
  install(CODE "execute_process(COMMAND \"/home/lijixiang/miniconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_nav\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_nav
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ucar_nav_generate_messages_py std_msgs_generate_messages_py)
endif()
