# Install headers
file(GLOB geometry_headers "*.h")
install(FILES ${geometry_headers} DESTINATION include/gtsam/geometry)

# Build tests
if(GTSAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
