# Install headers
file(GLOB base_headers "*.h")

# if(NOT USE_BOOST)
#     # 从列表中移除特定的头文件
#     list(REMOVE_ITEM base_headers "Testable.h")
# endif ()

install(FILES ${base_headers} DESTINATION include/gtsam/base)

file(GLOB base_headers_tree "treeTraversal/*.h")
install(FILES ${base_headers_tree} DESTINATION include/gtsam/base/treeTraversal)

# Build tests
if(GTSAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
