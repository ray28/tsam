# exclude certain files
# note the source dir on each 
set (tests_exclude "")
list (APPEND tests_exclude "testSerializationSlam.cpp")


# Build tests
gtsamAddTestsGlob(tests "test*.cpp" "${tests_exclude}" "gtsam")

if(MSVC)
	set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationSlam.cpp"
		APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()
