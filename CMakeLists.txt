#================================
# hltools - compiler toolchain
#================================

cmake_minimum_required(VERSION 3.21)

project(hltools
    DESCRIPTION "halflife map compiler toolchain"
    LANGUAGES CXX
)

include(CTest)
include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# keep the windows release self contained instead of requiring the visual c++
# redistributable beside the executable
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY
        "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# keep runnable artifacts with the source tree instead of burying them in
# whichever out of tree directory was selected for cmake's build files
set(HLTOOLS_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${HLTOOLS_RUNTIME_OUTPUT_DIRECTORY}")

# multi config generators such as visual studio otherwise append release,
# debug, and the other configuration names to the output directory
foreach(config IN LISTS CMAKE_CONFIGURATION_TYPES)
    string(TOUPPER "${config}" config_upper)
    set("CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_upper}"
        "${HLTOOLS_RUNTIME_OUTPUT_DIRECTORY}")
endforeach()

function(set_hltools_runtime_subdirectory target subdirectory)
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY
        "${HLTOOLS_RUNTIME_OUTPUT_DIRECTORY}/${subdirectory}")
    foreach(config IN LISTS CMAKE_CONFIGURATION_TYPES)
        string(TOUPPER "${config}" config_upper)
        set_target_properties(${target} PROPERTIES
            "RUNTIME_OUTPUT_DIRECTORY_${config_upper}"
            "${HLTOOLS_RUNTIME_OUTPUT_DIRECTORY}/${subdirectory}")
    endforeach()
endfunction()

function(add_hltools_test target)
    add_executable(${target} ${ARGN})
    set_hltools_runtime_subdirectory(${target} tests)
endfunction()

# version + short git commit stamped into the tool banner (captured at configure
# time; reconfigure to refresh the hash)
set(HLTOOLS_VERSION "1.23.0")
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE HLTOOLS_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT HLTOOLS_COMMIT)
    set(HLTOOLS_COMMIT "unknown")
endif()
add_compile_definitions(
    HLTOOLS_VERSION="${HLTOOLS_VERSION}"
    HLTOOLS_COMMIT="${HLTOOLS_COMMIT}"
)

# embedded map sources use a standards compliant zip tail; keep libzip and its
# zlib backend static so the command line tool remains a single file binary
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(ZLIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    zlib
    URL https://github.com/madler/zlib/archive/refs/tags/v1.3.1.tar.gz
    URL_HASH SHA256=17e88863f3600672ab49182f217281b6fc4d3c762bde361935e436a95214d05c
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(zlib)
set_target_properties(zlib PROPERTIES EXCLUDE_FROM_ALL TRUE)
if(NOT TARGET ZLIB::ZLIB)
    add_library(ZLIB::ZLIB ALIAS zlibstatic)
endif()
set(ZLIB_FOUND TRUE)
set(ZLIB_INCLUDE_DIRS "${zlib_SOURCE_DIR};${zlib_BINARY_DIR}")
set(ZLIB_INCLUDE_DIR "${zlib_SOURCE_DIR};${zlib_BINARY_DIR}")
set(ZLIB_LIBRARIES zlibstatic)
set(ZLIB_LIBRARY zlibstatic)
set(ZLIB_VERSION_STRING "1.3.1")

set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BUILD_REGRESS OFF CACHE BOOL "" FORCE)
set(BUILD_OSSFUZZ OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_DOC OFF CACHE BOOL "" FORCE)
set(LIBZIP_DO_INSTALL OFF CACHE BOOL "" FORCE)
set(ENABLE_BZIP2 OFF CACHE BOOL "" FORCE)
set(ENABLE_LZMA OFF CACHE BOOL "" FORCE)
set(ENABLE_ZSTD OFF CACHE BOOL "" FORCE)
set(ENABLE_OPENSSL OFF CACHE BOOL "" FORCE)
set(ENABLE_GNUTLS OFF CACHE BOOL "" FORCE)
set(ENABLE_MBEDTLS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    libzip
    URL https://github.com/nih-at/libzip/releases/download/v1.11.4/libzip-1.11.4.tar.gz
    URL_HASH SHA256=82e9f2f2421f9d7c2466bbc3173cd09595a88ea37db0d559a9d0a2dc60dc722e
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(libzip)

if(WIN32)
    enable_language(RC)

    if(NOT HLTOOLS_VERSION MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
        message(FATAL_ERROR
            "HLTOOLS_VERSION must contain three numeric components")
    endif()
    set(HLTOOLS_VERSION_MAJOR "${CMAKE_MATCH_1}")
    set(HLTOOLS_VERSION_MINOR "${CMAKE_MATCH_2}")
    set(HLTOOLS_VERSION_PATCH "${CMAKE_MATCH_3}")

    function(add_hltools_windows_version target description)
        set(HLTOOLS_FILE_DESCRIPTION "${description}")
        set(HLTOOLS_INTERNAL_NAME "${target}")
        set(HLTOOLS_ORIGINAL_FILENAME "${target}.exe")
        set(version_resource_dir
            "${CMAKE_CURRENT_BINARY_DIR}/generated/version")
        set(version_resource
            "${version_resource_dir}/${target}_version.rc")

        file(MAKE_DIRECTORY "${version_resource_dir}")
        configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/windows_version.rc.in"
            "${version_resource}"
            @ONLY
        )
        target_sources(${target} PRIVATE "${version_resource}")
    endfunction()
else()
    function(add_hltools_windows_version target description)
    endfunction()
endif()

if(MSVC)
    add_compile_options(/W3)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
    add_compile_options(-Wall -O2 -fno-strict-aliasing)
endif()

#================
# layers (bottom up) header only today; each grows a cpp as modules land,
# at which point it turns from interface into a static library
#================

# common: platform, logging, errors, filesystem - no domain knowledge
add_library(hltools_common STATIC
    src/common/build_info.cpp
    src/common/log.cpp
    src/common/error.cpp
    src/common/filesystem.cpp
    src/common/threads.cpp
    src/common/cmdline.cpp
    src/common/progress.cpp
)
target_include_directories(hltools_common PUBLIC src)

# math: templated vector, plane, winding, bounding box (float and double)
add_library(hltools_math INTERFACE)
target_include_directories(hltools_math INTERFACE src)
target_link_libraries(hltools_math INTERFACE hltools_common)

# format: bsp, wad, bmp, miptex, entity lump, and rad text i/o
add_library(hltools_format STATIC
    src/format/bmp/file.cpp
    src/format/bsp/entity_lump.cpp
    src/format/bsp/face_extents.cpp
    src/format/bsp/file.cpp
    src/format/bsp/lightmap_atlas.cpp
    src/format/bsp/texture_lump.cpp
    src/format/bsp/usage_chart.cpp
    src/format/zip/archive.cpp
    src/format/image/quantize.cpp
    src/format/map/document.cpp
    src/format/mdl/goldsrc_mdl.cpp
    src/format/mdl/source_mdl.cpp
    src/format/miptex/texture.cpp
    src/format/rad/texlights.cpp
    src/format/vbsp/file.cpp
    src/format/vbsp/pakfile.cpp
    src/format/vmt/vmt.cpp
    src/format/vpk/vpk.cpp
    src/format/vtf/vtf.cpp
    src/format/wad/archive.cpp
)
target_include_directories(hltools_format PUBLIC src)
target_link_libraries(hltools_format PUBLIC hltools_common zip)

# shared compiler option registry consumers and info_compile_parameters
# resolution for unified and individual stage commands
add_library(hltools_compile_parameters STATIC
    src/cli/tools/compile_parameters.cpp
)
target_include_directories(hltools_compile_parameters PUBLIC src)
target_link_libraries(hltools_compile_parameters
    PUBLIC hltools_format hltools_common)

# decompile: read only bsp tree reconstruction and valve 220 map writing
add_library(hltools_decompile STATIC
    src/decompile/map_decompiler.cpp
    src/decompile/source_displacements.cpp
    src/decompile/source_entities.cpp
    src/decompile/source_map_decompiler.cpp
    src/decompile/source_materials.cpp
    src/decompile/source_models.cpp
    src/decompile/source_skybox.cpp
)
target_include_directories(hltools_decompile PUBLIC src)
target_link_libraries(hltools_decompile PUBLIC hltools_format hltools_math hltools_common)

# model asset generators and converters
add_library(hltools_model STATIC
    src/model/skybox_model.cpp
)
target_include_directories(hltools_model PUBLIC src)
target_link_libraries(hltools_model PUBLIC hltools_format hltools_common)

# vis: portal file loading, pvs flow, and visibility lump compression
add_library(hltools_vis STATIC
    src/vis/vis.cpp
    src/vis/flow.cpp
)
target_include_directories(hltools_vis PUBLIC src)
target_link_libraries(hltools_vis PUBLIC hltools_format hltools_math hltools_common)
# the reference vis build used whole program optimization (msvc /gl + ltcg);
# without it the port loses measurable time in the leafthread hot path
set_target_properties(hltools_vis PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)

# csg: map parsing and brush processing
add_library(hltools_csg STATIC
    src/csg/bsp_output.cpp
    src/csg/brush.cpp
    src/csg/brush_union.cpp
    src/csg/csg.cpp
    src/csg/entity_output_fixups.cpp
    src/csg/hull_file.cpp
    src/csg/map_parser.cpp
    src/csg/map_post.cpp
    src/csg/textures.cpp
    src/csg/wad_path.cpp
)
target_include_directories(hltools_csg PUBLIC src)
target_compile_definitions(hltools_csg PUBLIC HLTOOLS_DOUBLE_PRECISION)
target_link_libraries(hltools_csg PUBLIC hltools_format hltools_math hltools_common)

# bsp: solid tree build, portals, fill, tjunc, edges, and bsp writing
add_library(hltools_bsp STATIC
    src/bsp/brush.cpp
    src/bsp/face.cpp
    src/bsp/inputs.cpp
    src/bsp/merge.cpp
    src/bsp/pack.cpp
    src/bsp/bsp.cpp
    src/bsp/brink.cpp
    src/bsp/outside.cpp
    src/bsp/portals.cpp
    src/bsp/solid_bsp.cpp
    src/bsp/surfaces.cpp
    src/bsp/tjunc.cpp
    src/bsp/write_bsp.cpp
)
target_include_directories(hltools_bsp PUBLIC src)
target_compile_definitions(hltools_bsp PUBLIC HLTOOLS_DOUBLE_PRECISION)
target_link_libraries(hltools_bsp PUBLIC hltools_format hltools_math hltools_common)

# rad: radiosity lighting float precision like the reference build (no
# hltools_double_precision)
add_library(hltools_rad STATIC
    src/rad/compress.cpp
    src/rad/embed_lightmap.cpp
    src/rad/final.cpp
    src/rad/gpu_gather.cpp
    src/rad/lerp.cpp
    src/rad/light_budget.cpp
    src/rad/lightmap.cpp
    src/rad/nomatrix.cpp
    src/rad/patches.cpp
    src/rad/rad.cpp
    src/rad/position.cpp
    src/rad/sparse.cpp
    src/rad/studio.cpp
    src/rad/textures.cpp
    src/rad/trace.cpp
    src/rad/transfers.cpp
    src/rad/transparency.cpp
    src/rad/util.cpp
    src/rad/vismatrix.cpp
)
target_include_directories(hltools_rad PUBLIC src)
target_link_libraries(hltools_rad PUBLIC hltools_format hltools_math hltools_common)
# the reference rad target is built with interprocedural optimization (msvc
# ltcg / /gl) ltcg reassociates and contracts floating point across
# translation units, so the lightmap output only matches the reference build
# bit for bit when rad is compiled with the same setting the inlined math
# lives in the rad translation units, so ipo on this library covers it
set_target_properties(hltools_rad PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)

# gpu lighting backend: vulkan compute, glsl
# kernels compiled to spir v by glslc at build time and embedded as byte
# arrays the vulkan loader libraries are opened at runtime, so
# the only build dependencies are glslc (vulkan sdk, distro package, or an
# android ndk) vulkan headers is fetched at configure time into the build
# tree; the loader itself is not a link time dependency
# hltools_gpu=off builds the toolchain without the optional backend
option(HLTOOLS_GPU "build the vulkan gpu lighting backend" ON)

if(HLTOOLS_GPU)
    file(GLOB NDK_SHADER_TOOLS
        "$ENV{LOCALAPPDATA}/Android/Sdk/ndk/*/shader-tools/windows-x86_64"
        "$ENV{HOME}/Android/Sdk/ndk/*/shader-tools/linux-x86_64")
    find_program(GLSLC_EXECUTABLE glslc
        HINTS "$ENV{VULKAN_SDK}/bin" ${NDK_SHADER_TOOLS}
    )
    if(NOT GLSLC_EXECUTABLE)
        message(WARNING "HLTOOLS_GPU: glslc not found; gpu backend disabled")
        set(HLTOOLS_GPU OFF)
    endif()
endif()

if(HLTOOLS_GPU)
    set(FETCHCONTENT_BASE_DIR "${CMAKE_BINARY_DIR}/_extern")
    FetchContent_Declare(
        vulkan_headers
        GIT_REPOSITORY https://github.com/KhronosGroup/Vulkan-Headers.git
        GIT_TAG b379292b2ab6df5771ba9870d53cf8b2c9295daf
        GIT_SHALLOW FALSE
    )
    FetchContent_MakeAvailable(vulkan_headers)

    set(RAD_GPU_GEN_DIR ${CMAKE_BINARY_DIR}/generated/rad_gpu)
    set(RAD_GPU_SHADER_DIR ${CMAKE_SOURCE_DIR}/src/rad/gpu/shaders)
    # src compiles to name_spirvh as symbol; extra args are passed to glslc
    # (variant defines like -dfloat_normalize)
    function(rad_gpu_kernel_src SRC NAME SYMBOL)
        add_custom_command(
            OUTPUT ${RAD_GPU_GEN_DIR}/${NAME}_spirv.h
            COMMAND ${CMAKE_COMMAND} -E make_directory ${RAD_GPU_GEN_DIR}
            COMMAND ${GLSLC_EXECUTABLE} --target-env=vulkan1.0 -O ${ARGN}
                    -o ${RAD_GPU_GEN_DIR}/${NAME}.spv
                    ${RAD_GPU_SHADER_DIR}/${SRC}.comp
            COMMAND ${CMAKE_COMMAND}
                    -DINPUT=${RAD_GPU_GEN_DIR}/${NAME}.spv
                    -DOUTPUT=${RAD_GPU_GEN_DIR}/${NAME}_spirv.h
                    -DSYMBOL=${SYMBOL}
                    -P ${CMAKE_SOURCE_DIR}/cmake/embed_binary.cmake
            DEPENDS ${RAD_GPU_SHADER_DIR}/${SRC}.comp
                    ${RAD_GPU_SHADER_DIR}/trace_lib.glsl
                    ${CMAKE_SOURCE_DIR}/cmake/embed_binary.cmake
            COMMENT "glslc: ${SRC}.comp -> ${NAME} spir-v"
        )
    endfunction()
    function(rad_gpu_kernel NAME SYMBOL)
        rad_gpu_kernel_src(${NAME} ${NAME} ${SYMBOL})
    endfunction()
    rad_gpu_kernel(trace_bsp g_trace_bsp_spirv)
    rad_gpu_kernel(gather g_gather_spirv)
    rad_gpu_kernel_src(gather gather_f32 g_gather_f32_spirv -DFLOAT_NORMALIZE)
    rad_gpu_kernel(formfactor g_formfactor_spirv)

    add_library(hltools_rad_gpu STATIC
        src/rad/gpu/gpu.cpp
        ${RAD_GPU_GEN_DIR}/trace_bsp_spirv.h
        ${RAD_GPU_GEN_DIR}/gather_spirv.h
        ${RAD_GPU_GEN_DIR}/gather_f32_spirv.h
        ${RAD_GPU_GEN_DIR}/formfactor_spirv.h
    )
    target_include_directories(hltools_rad_gpu PUBLIC src
        PRIVATE ${RAD_GPU_GEN_DIR})
    target_link_libraries(hltools_rad_gpu
        PUBLIC hltools_common
        PRIVATE Vulkan::Headers)
    if(NOT WIN32)
        target_link_libraries(hltools_rad_gpu PRIVATE ${CMAKE_DL_LIBS})
    endif()

    # rad's -gpu path (gpu_gathercpp) compiles against the backend; without
    # hltools_gpu it builds as stubs that warn and fall back to the cpu
    target_compile_definitions(hltools_rad PRIVATE HLTOOLS_GPU_ENABLED)
    target_link_libraries(hltools_rad PUBLIC hltools_rad_gpu)

endif()

# hltools: the single binary toolkit links every stage library; the whole
# target compiles with hltools_double_precision (propagated from csg/bsp), which
# is safe because the vis/rad public option structs are precision neutral
# ipo preserves the reference compiler's whole-program optimization behavior
add_executable(hltools
    src/cli/hltools/main.cpp
    src/cli/hltools/compile.cpp
    src/cli/tools/csg_tool.cpp
    src/cli/tools/bsp_tool.cpp
    src/cli/tools/vis_tool.cpp
    src/cli/tools/rad_tool.cpp
    src/cli/tools/bsp_info_tool.cpp
    src/cli/tools/lightmap_tool.cpp
    src/cli/tools/wad_tool.cpp
    src/cli/tools/decompile_tool.cpp
    src/cli/tools/model_tool.cpp
)
target_link_libraries(hltools PRIVATE hltools_csg hltools_bsp hltools_vis hltools_rad
    hltools_decompile hltools_model hltools_compile_parameters)
add_hltools_windows_version(hltools "${PROJECT_DESCRIPTION}")
set_target_properties(hltools PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
