#gtsamAddTestsGlob(slam "test*.cpp" "" "gtsam")

set(exclude_files "")
if(NOT USE_BOOST_DEV)
    # 从列表中移除特定的头文件
    list(APPEND exclude_files "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationDataset.cpp")
    list(APPEND exclude_files "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationInSlam.cpp")
endif ()

gtsamAddTestsGlob(slam "test*.cpp" "${exclude_files}" "gtsam")