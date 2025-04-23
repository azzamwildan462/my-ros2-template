# Declare header-only interface library
add_library(global_lib INTERFACE)

# Expose the include directory
target_include_directories(global_lib INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Optional install support
install(DIRECTORY include/ DESTINATION include)


# Optional: add a source file to the library (if exists, i just don't like it xd)
# add_library(global_lib
#     lib/global_lib.cpp
# )
# target_include_directories(global_lib PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
# )
# install(TARGETS global_lib DESTINATION lib)

