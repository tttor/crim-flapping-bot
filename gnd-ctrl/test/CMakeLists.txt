## Target
set(TEST_SRCS test-serial-2.cpp)
add_executable(./bin/test-serial-2 ${TEST_SRCS})

## Link libraries
set(BOOST_LIBS thread date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
target_link_libraries(./bin/test-serial-2 ${Boost_LIBRARIES})

find_package(Threads REQUIRED)
target_link_libraries(./bin/test-serial-2 ${CMAKE_THREAD_LIBS_INIT} )

target_link_libraries (./bin/test-serial-2 ${EXTRA_LIBS})
