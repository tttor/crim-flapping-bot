cmake_minimum_required(VERSION 2.6)
project(TEST)

## Include dir
include_directories("${PROJECT_SOURCE_DIR}/inc")
include_directories("../common/inc")

include_directories ("${PROJECT_SOURCE_DIR}/lib/serial-port")
add_subdirectory (lib/serial-port)
set (EXTRA_LIBS ${EXTRA_LIBS} TimeoutSerial)
  
## Target
add_subdirectory(test)
