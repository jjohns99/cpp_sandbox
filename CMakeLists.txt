cmake_minimum_required(VERSION 3.5)
project(test)

add_compile_options(-std=c++11 -fvisibility=hidden)
#set(PYBIND11_CPP_STANDARD -std=c++11)
#set(PYBIND11_PYTHON_VERSION 3.5)



#find_package(Python COMPONENTS Interpreter Development)
#find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
#find_package(OpenCV REQUIRED)
#find_package(holodeck_cpp_bindings REQUIRED)


include_directories(
#	${Python_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR})
#	${OpenCV_INCLUDE_DIRS}
#        ${HOLODECK_CPP_BINDINGS_INCLUDE_DIRS})



add_executable(example src/main.cpp)



#target_link_libraries(example PRIVATE 
#	${OpenCV_LIBRARIES}
#	${HOLODECK_CPP_BINDINGS_LIBRARIES})
