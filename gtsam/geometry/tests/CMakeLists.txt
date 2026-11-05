#gtsamAddTestsGlob(geometry "test*.cpp" "" "gtsam")

set(exclude_files "")
if(NOT USE_BOOST_DEV)
    # 从列表中移除特定的头文件
    list(APPEND exclude_files "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationGeometry.cpp")
endif ()

gtsamAddTestsGlob(geometry "test*.cpp" "${exclude_files}" "gtsam")