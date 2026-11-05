# Install headers
file(GLOB sam_headers "*.h")
install(FILES ${sam_headers} DESTINATION include/gtsam/sam)

# Build tests
if(GTSAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
