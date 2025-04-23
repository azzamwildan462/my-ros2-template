# ==== C++ Section ====
add_executable(master src/master.cpp)
target_link_libraries(master global_lib)

target_include_directories(master PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(master PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)

# ==== Python Section ====
file(GLOB PY_SCRIPTS scripts/*.py)

foreach(py ${PY_SCRIPTS})
    get_filename_component(script_name ${py} NAME)
    add_custom_command(
        OUTPUT ${CMAKE_SOURCE_DIR}/bin/${script_name}
        COMMAND ${CMAKE_COMMAND} -E create_symlink
                ${py}
                ${CMAKE_SOURCE_DIR}/bin/${script_name}
        DEPENDS ${py}
    )
    list(APPEND PY_LINKS ${CMAKE_SOURCE_DIR}/bin/${script_name})
endforeach()

add_custom_target(python_symlinks ALL DEPENDS ${PY_LINKS})
