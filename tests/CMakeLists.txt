include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
FetchContent_MakeAvailable(googletest)

add_executable(test_boolean_op test_boolean_op.cpp)
target_link_libraries(test_boolean_op
  PRIVATE
  rectilinear3d_boolean
  GTest::gtest_main
  )
add_test(NAME test_boolean_op COMMAND test_boolean_op)
