cmake_minimum_required(VERSION 3.16)
project(policy_bridge LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEFAULT_UNITREE_SDK2_DIR "/home/unitree/yixuan/yichao-deploy/Score_Real/unitree_sdk2")
set(UNITREE_SDK2_DIR "${DEFAULT_UNITREE_SDK2_DIR}" CACHE PATH "Path to installed or source unitree_sdk2 tree")
set(UNITREE_SDK2_INCLUDE_DIR "${UNITREE_SDK2_DIR}/include")

find_library(UNITREE_SDK2_LIB
  NAMES unitree_sdk2 libunitree_sdk2.a
  HINTS
    ${UNITREE_SDK2_DIR}/lib
    ${UNITREE_SDK2_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}
    ${UNITREE_SDK2_DIR}/build/lib
    ${UNITREE_SDK2_DIR}/build/lib/${CMAKE_SYSTEM_PROCESSOR}
)
find_library(DDSC_LIB
  NAMES ddsc libddsc.so
  HINTS
    ${UNITREE_SDK2_DIR}/lib
    ${UNITREE_SDK2_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}
    ${UNITREE_SDK2_DIR}/build/lib
    ${UNITREE_SDK2_DIR}/thirdparty/lib
    ${UNITREE_SDK2_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}
)
find_library(DDSCXX_LIB
  NAMES ddscxx libddscxx.so
  HINTS
    ${UNITREE_SDK2_DIR}/lib
    ${UNITREE_SDK2_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}
    ${UNITREE_SDK2_DIR}/build/lib
    ${UNITREE_SDK2_DIR}/thirdparty/lib
    ${UNITREE_SDK2_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}
)

if(NOT EXISTS ${UNITREE_SDK2_INCLUDE_DIR})
  message(FATAL_ERROR "unitree_sdk2 headers not found under ${UNITREE_SDK2_INCLUDE_DIR}")
endif()

if(NOT UNITREE_SDK2_LIB)
  message(FATAL_ERROR
    "Could not locate libunitree_sdk2 for bridge build.\n"
    "Point UNITREE_SDK2_DIR to a built/install tree, or build the SDK first."
  )
endif()

if(NOT DDSC_LIB OR NOT DDSCXX_LIB)
  message(FATAL_ERROR
    "Could not locate CycloneDDS runtime libs (ddsc/ddscxx).\n"
    "Build/install unitree_sdk2 and its thirdparty CycloneDDS libs first."
  )
endif()

find_package(Threads REQUIRED)

add_library(unitree_sdk2_bridge_deps INTERFACE)
target_include_directories(unitree_sdk2_bridge_deps INTERFACE
  ${UNITREE_SDK2_INCLUDE_DIR}
  ${UNITREE_SDK2_DIR}/thirdparty/include
  ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
)
target_link_libraries(unitree_sdk2_bridge_deps INTERFACE
  ${UNITREE_SDK2_LIB}
  ${DDSC_LIB}
  ${DDSCXX_LIB}
  Threads::Threads
)

find_program(CYCLONEDDS_IDLC idlc)
if(NOT CYCLONEDDS_IDLC)
  find_program(CYCLONEDDS_IDLC idlc
    HINTS /home/unitree/unitree_ros2/cyclonedds_ws/install/cyclonedds/bin)
endif()

set(BRIDGE_IDL ${CMAKE_CURRENT_SOURCE_DIR}/idl/policy_bridge.idl)
set(BRIDGE_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${BRIDGE_GENERATED_DIR})

if(CYCLONEDDS_IDLC)
  add_custom_command(
    OUTPUT ${BRIDGE_GENERATED_DIR}/policy_bridge.h ${BRIDGE_GENERATED_DIR}/policy_bridge.c
    COMMAND ${CYCLONEDDS_IDLC} -l c -o ${BRIDGE_GENERATED_DIR} ${BRIDGE_IDL}
    DEPENDS ${BRIDGE_IDL}
    COMMENT "Generating C bridge DDS types with idlc"
    VERBATIM
  )
  add_library(policy_bridge_idl STATIC ${BRIDGE_GENERATED_DIR}/policy_bridge.c)
  target_include_directories(policy_bridge_idl PUBLIC
    ${BRIDGE_GENERATED_DIR}
    ${UNITREE_SDK2_DIR}/thirdparty/include
  )
  target_link_libraries(policy_bridge_idl PUBLIC ${DDSC_LIB})
else()
  message(WARNING "CycloneDDS idlc not found. DDS bridge transport must be generated manually.")
endif()

add_executable(cpp_bridge_main cpp_bridge_main.cpp)
target_link_libraries(cpp_bridge_main PRIVATE unitree_sdk2_bridge_deps)

if(TARGET policy_bridge_idl)
  target_link_libraries(cpp_bridge_main PRIVATE policy_bridge_idl)
endif()
