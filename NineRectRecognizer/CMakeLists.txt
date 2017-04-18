cmake_minimum_required(VERSION 2.8)
#project name
PROJECT(NineRectRecognition)
#head file path
INCLUDE_DIRECTORIES(include)
#source file path
#AUX_SOURCE_DIRECTORY(src DIR_SRCS)
#set environment variable
#SET(opencv_test ${DIR_SRCS})
#find package
FIND_PACKAGE(OpenCV REQUIRED)
#add executable file
ADD_EXECUTABLE(main main.cpp)
#add link librbry
TARGET_LINK_LIBRARIES(main ${OpenCV_LIBS})
