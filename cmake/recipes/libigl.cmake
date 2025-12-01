# libigl (https://github.com/libigl/libigl)
# License: MPL-2.0
if(TARGET igl::core)
    return()
endif()

message(STATUS "Third-party: creating target 'igl::core'")

set(LIBIGL_PREDICATES ON CACHE BOOL "Use exact predicates" FORCE)

include(eigen)

include(CPM)
CPMAddPackage("gh:libigl/libigl@2.4.0")

# Folder name for IDE
foreach(target_name IN ITEMS core predicates)
    set_target_properties(igl_${target_name} PROPERTIES FOLDER "ThirdParty/libigl")
endforeach()

# Suppress warnings for libigl targets
if(TARGET igl::core)
    target_compile_options(igl_core INTERFACE -w)
endif()
if(TARGET igl::predicates)
    target_compile_options(igl_predicates INTERFACE -w)
endif()
