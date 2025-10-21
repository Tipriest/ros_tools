# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)

# 定义项目名称和版本号
set(PROJECT_N pinocchio_load_model) # 项目名称
project(${PROJECT_N} VERSION 1.0)    # 项目版本号

# 设置C++标准为C++17，并强制要求支持该标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 设置构建类型为Debug模式
set(CMAKE_BUILD_TYPE Debug)

# 定义一个宏，用于指定Pinocchio模型目录
add_compile_definitions(PINOCCHIO_MODEL_DIR=\"/opt/openrobots/share\")

# 搜索项目源文件
# 递归查找src目录下与项目名称匹配的源文件
file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/${PROJECT_N}.cpp)

# 查找Pinocchio和Eigen3库
find_package(pinocchio REQUIRED) # 查找Pinocchio库，若未找到则报错
find_package(Eigen3 REQUIRED)    # 查找Eigen3库，若未找到则报错

# 定义一个宏，用于递归获取所有包含指定扩展名的头文件目录
macro(header_directories return_list includes_base_folder extention )
    # 递归查找指定目录下的所有指定扩展名的文件
    file(GLOB_RECURSE new_list ${includes_base_folder}/*.${extention})
    set(dir_list "")
    foreach(file_path ${new_list})
        # 获取每个文件的目录路径
        get_filename_component(dir_path ${file_path} PATH)
        set(dir_list ${dir_list} ${dir_path})
    endforeach()
    # 去重，确保目录列表中没有重复项
    list(REMOVE_DUPLICATES dir_list)
    set(${return_list} ${dir_list})
endmacro()

# 使用上述宏，获取include目录下所有.hpp文件的目录
header_directories(INCLUDES ${PROJECT_SOURCE_DIR}/include/ hpp)

# 打印找到的源文件列表
message("src files:")
foreach(file ${SRCS})
        message(STATUS ${file})
endforeach()

# 打印找到的头文件目录列表
message("include directories:")
foreach(dir ${INCLUDES})
        message(STATUS ${dir})
endforeach()

# 添加可执行文件
# 将找到的源文件添加到可执行文件的构建目标中
add_executable(${PROJECT_N} ${SRCS})

# 设置可执行文件的头文件搜索路径
target_include_directories(${PROJECT_N} PUBLIC 
    include                     # 项目本地的include目录
    ${PINOCCHIO_INCLUDE_DIRS}   # Pinocchio库的头文件目录
    ${EIGEN3_INCLUDE_DIR}       # Eigen3库的头文件目录
)

# 链接Pinocchio和Eigen3库
target_link_libraries(${PROJECT_N}
    PRIVATE
        pinocchio::pinocchio    # Pinocchio库
        Eigen3::Eigen          # Eigen3库
)
