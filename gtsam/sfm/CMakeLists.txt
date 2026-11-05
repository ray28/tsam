# Install headers
file(GLOB sfm_headers "*.h")
install(FILES ${sfm_headers} DESTINATION include/gtsam/sfm)

# Build tests
if(GTSAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
