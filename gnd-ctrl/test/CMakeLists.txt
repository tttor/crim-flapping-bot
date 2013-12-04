## Target
#set(TEST_SRCS test-serial-2.cpp)
set(TEST_SRCS test-receiving-packet.cpp)
add_executable(./bin/test ${TEST_SRCS})

## Link libraries
set(BOOST_LIBS thread date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
target_link_libraries(./bin/test ${Boost_LIBRARIES})

find_package(Threads REQUIRED)
target_link_libraries(./bin/test ${CMAKE_THREAD_LIBS_INIT} )

target_link_libraries (./bin/test ${EXTRA_LIBS})
