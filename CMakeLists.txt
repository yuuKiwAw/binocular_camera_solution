cmake_minimum_required(VERSION 3.24)
project(binocular_camera_solution)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4.5.5 REQUIRED)

set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)


# 找到所有含有mian函数的cpp文件，保存在mians里
file(
    GLOB_RECURSE mains RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/stereo_calib.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/stereo_match.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/stereo_match_video.cpp"
)
# 循环遍历每个含main的文件，分别编译生成exe文件
foreach(mainfile IN LISTS mains)
    # Get file name without directory
    get_filename_component(mainname ${mainfile} NAME_WE)
    add_executable(${mainname} ${mainfile})
    # 添加库文件目录
    target_link_libraries(${mainname} PRIVATE ${OpenCV_LIBS})# 链接库文件
endforeach()