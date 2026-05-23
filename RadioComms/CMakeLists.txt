cmake_minimum_required(VERSION 3.28)
project(HoustonRadioComms VERSION 0.1 LANGUAGES C CXX)

add_executable(radio_main radio_main.cpp)
target_compile_features(radio_main PRIVATE cxx_std_20)

target_compile_definitions(radio_main PRIVATE _WIN32_WINNT=0x0A00)

target_include_directories(radio_main PRIVATE .)

# We need Boost for ASIO and Turtleford for the protobuf encoding
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../Turtleford TurtlefordBuild)

include(FetchContent)
set(BOOST_ENABLE_CMAKE ON)
set(BOOST_INCLUDE_LIBRARIES asio system)
FetchContent_Declare(
    Boost
    GIT_REPOSITORY https://github.com/boostorg/boost.git
    GIT_TAG boost-1.90.0
)
FetchContent_MakeAvailable(Boost)

target_link_libraries(radio_main PRIVATE Boost::asio Boost::system Turtleford)
