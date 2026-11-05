# Install headers
set(subdir hybrid)
file(GLOB hybrid_headers "*.h")
# FIXME: exclude headers
install(FILES ${hybrid_headers} DESTINATION include/gtsam/hybrid)

# Add all tests
if(GTSAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
