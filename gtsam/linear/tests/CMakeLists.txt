set(exclude_files "")
if(NOT USE_BOOST_DEV)
	# 从列表中移除特定的头文件
	list(APPEND exclude_files "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationLinear.cpp")
#	list(APPEND exclude_files "${CMAKE_CURRENT_SOURCE_DIR}/testJacobianFactor.cpp")
endif ()

gtsamAddTestsGlob(linear "test*.cpp" "${exclude_files}" "gtsam")

# gtsamAddTestsGlob(linear "test*.cpp" "" "gtsam")

if(MSVC)
	set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationLinear.cpp"
			APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()
