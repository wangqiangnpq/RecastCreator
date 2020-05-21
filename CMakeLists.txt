# CMakeList.txt: AIServer 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)
project (RecastCreator C CXX)

#这个就是定义各个版本对应的后缀，例如d，debug版本后缀，当然你想定义为其他，	#自己修改这块就可以了。
SET(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add a postfix, usually Debug")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a postfix, usually Release")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "rd" CACHE STRING "add a postfix, usually RelWithDebInfo")
SET(CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add a postfix, usually MinSizeRel")

#判断自己的编译状态
IF(CMAKE_BUILD_TYPE MATCHES "Debug")
	MESSAGE(STATUS "current compilation type: Debug ")
	set(BIN_PATH ${PROJECT_SOURCE_DIR}/build)
	set(LIB_PATH ${PROJECT_SOURCE_DIR}/build)
ELSEIF(CMAKE_BUILD_TYPE MATCHES "Release")
	MESSAGE(STATUS "current compilation type: Release ")
	set(BIN_PATH ${PROJECT_SOURCE_DIR}/build)
	set(LIB_PATH ${PROJECT_SOURCE_DIR}/build)
ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
	MESSAGE(STATUS "current compilation type: RelWithDebInfo")
	set(BIN_PATH ${PROJECT_SOURCE_DIR}/build)
	set(LIB_PATH ${PROJECT_SOURCE_DIR}/build)
ELSEIF(CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
	MESSAGE(STATUS "current compilation type: MinSizeRel")
	set(BIN_PATH ${PROJECT_SOURCE_DIR}/build)
	set(LIB_PATH ${PROJECT_SOURCE_DIR}/build)
ENDIF()

#设置库文件存放路径
set(LIBRARY_OUTPUT_PATH ${LIB_PATH})
#设置执行文件存放路径
set(EXECUTABLE_OUTPUT_PATH ${BIN_PATH})

# 并将名称保存到 DIR_SRCS 变量
file(GLOB_RECURSE DIR_SRCS ./*.cpp ./*.h ./*.hpp)
message(STATUS ${DIR_SRCS})
# 将源代码添加到此项目的可执行文件。
add_executable(${PROJECT_NAME} ${DIR_SRCS})