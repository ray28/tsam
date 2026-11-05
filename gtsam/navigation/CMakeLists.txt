# Install headers
file(GLOB navigation_headers "*.h")
install(FILES ${navigation_headers} DESTINATION include/gtsam/navigation)

# Add all tests
if(GTSAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
