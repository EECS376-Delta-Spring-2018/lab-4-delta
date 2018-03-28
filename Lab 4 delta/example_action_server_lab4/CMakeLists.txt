cmake_minimum_required(VERSION 2.8.3)
project(example_action_server_lab4)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Executables
cs_add_executable(example_action_server_lab4 src/example_action_server.cpp)
cs_add_executable(example_action_client_lab4 src/example_action_client.cpp)
cs_install()
cs_export()
    
