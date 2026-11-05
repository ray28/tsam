# Install headers
file(GLOB linear_headers "*.h")
install(FILES ${linear_headers} DESTINATION include/gtsam/linear)

# Build tests
if(GTSAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
