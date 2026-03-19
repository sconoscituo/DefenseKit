include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(test_ballistics test_ballistics.cpp)
target_link_libraries(test_ballistics defensekit_lib GTest::gtest_main)

add_executable(test_fft test_fft.cpp)
target_link_libraries(test_fft defensekit_lib GTest::gtest_main)

add_executable(test_kalman test_kalman.cpp)
target_link_libraries(test_kalman defensekit_lib GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_ballistics)
gtest_discover_tests(test_fft)
gtest_discover_tests(test_kalman)
