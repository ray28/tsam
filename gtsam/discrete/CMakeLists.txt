# Install headers
set(subdir discrete)
file(GLOB discrete_headers "*.h")
# FIXME: exclude headers
install(FILES ${discrete_headers} DESTINATION include/gtsam/discrete)

# Add all tests
if(GTSAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
