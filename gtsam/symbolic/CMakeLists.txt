# Install headers
file(GLOB symbolic_headers "*.h")
install(FILES ${symbolic_headers} DESTINATION include/gtsam/symbolic)

# Build tests
if(GTSAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
