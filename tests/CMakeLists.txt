include(FetchContent)

set(FETCHCONTENT_BASE_DIR "${CMAKE_BINARY_DIR}/_extern")
FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG v2.5.3
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(doctest)
include("${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake")

set(HLTOOLS_TEST_SCRATCH_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/tests/_work")
set(HLTOOLS_REPOSITORY_ROOT "${CMAKE_SOURCE_DIR}/..")
file(MAKE_DIRECTORY "${HLTOOLS_TEST_SCRATCH_DIRECTORY}")

add_library(hltools_test_support INTERFACE)
target_include_directories(hltools_test_support INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/support")
target_compile_definitions(hltools_test_support INTERFACE
    HLTOOLS_TEST_SCRATCH_DIRECTORY="${HLTOOLS_TEST_SCRATCH_DIRECTORY}")
target_link_libraries(hltools_test_support INTERFACE doctest::doctest)

function(add_hltools_unit target)
    cmake_parse_arguments(TEST "" "PREFIX;LABEL" "SOURCES;LIBRARIES" ${ARGN})
    add_hltools_test(${target} ${TEST_SOURCES})
    target_link_libraries(${target} PRIVATE
        doctest::doctest_with_main
        hltools_test_support
        ${TEST_LIBRARIES})
    doctest_discover_tests(${target}
        TEST_PREFIX "${TEST_PREFIX}"
        WORKING_DIRECTORY "${HLTOOLS_REPOSITORY_ROOT}"
        PROPERTIES LABELS "${TEST_LABEL}"
        DISCOVERY_MODE PRE_TEST)
endfunction()

function(add_hltools_check target)
    cmake_parse_arguments(TEST "" "NAME" "SOURCES;LIBRARIES;ARGS;LABELS" ${ARGN})
    if(NOT TEST_NAME)
        message(FATAL_ERROR "add_hltools_check(${target}) requires NAME")
    endif()
    add_hltools_test(${target} ${TEST_SOURCES})
    target_link_libraries(${target} PRIVATE hltools_test_support ${TEST_LIBRARIES})
    add_test(NAME "${TEST_NAME}" COMMAND $<TARGET_FILE:${target}> ${TEST_ARGS})
    set_tests_properties("${TEST_NAME}" PROPERTIES
        WORKING_DIRECTORY "${HLTOOLS_REPOSITORY_ROOT}"
        LABELS "${TEST_LABELS}")
endfunction()

function(add_hltools_manual_check target)
    cmake_parse_arguments(TEST "" "" "SOURCES;LIBRARIES" ${ARGN})
    add_hltools_test(${target} ${TEST_SOURCES})
    target_link_libraries(${target} PRIVATE hltools_test_support ${TEST_LIBRARIES})
endfunction()

# unit suites use the lowercase api from support/testh each source file may
# contain several independently discovered test cases
add_hltools_unit(unit_common
    PREFIX "unit.common."
    SOURCES
        unit/common/binary_io.cpp
        unit/common/build_info.cpp
    LIBRARIES hltools_common
    LABEL unit.common)

add_hltools_unit(unit_cli
    PREFIX "unit.cli."
    SOURCES
        unit/cli/compile_parameters.cpp
    LIBRARIES hltools_compile_parameters
    LABEL unit.cli)

add_hltools_unit(unit_format
    PREFIX "unit.format."
    SOURCES
        unit/format/bsp_extensions.cpp
        unit/format/chunk_skins.cpp
        unit/format/entity_lump.cpp
        unit/format/mdl/goldsrc.cpp
        unit/format/quantize.cpp
        unit/format/rad_file.cpp
        unit/format/resample.cpp
        unit/format/mdl/source.cpp
        unit/format/phy/source.cpp
    LIBRARIES hltools_format
    LABEL unit.format)

add_hltools_unit(unit_port
    PREFIX "unit.port."
    SOURCES
        unit/decompile/source/displacements.cpp
        unit/decompile/source/entities.cpp
        unit/decompile/source/materials.cpp
        unit/decompile/source/models.cpp
        unit/decompile/source/skybox.cpp
    LIBRARIES hltools_port
    LABEL unit.port)

add_hltools_unit(unit_model
    PREFIX "unit.model."
    SOURCES
        unit/model/skybox_model.cpp
    LIBRARIES hltools_model
    LABEL unit.model)

add_hltools_unit(unit_math
    PREFIX "unit.math."
    SOURCES unit/math/winding.cpp
    LIBRARIES hltools_math
    LABEL unit.math)

add_hltools_unit(unit_bsp
    PREFIX "unit.bsp."
    SOURCES
        unit/bsp/leaf_content_conflicts.cpp
        unit/bsp/pack.cpp
        unit/bsp/subdivide_face.cpp
    LIBRARIES hltools_bsp
    LABEL unit.bsp)

add_hltools_unit(unit_csg
    PREFIX "unit.csg."
    SOURCES
        unit/csg/map_post.cpp
        unit/csg/map_parser.cpp
        unit/csg/textures.cpp
    LIBRARIES hltools_csg
    LABEL unit.csg)

add_hltools_unit(unit_rad
    PREFIX "unit.rad."
    SOURCES
        unit/rad/options.cpp
        unit/rad/unlit_textures.cpp
        ../src/cli/tools/rad_tool.cpp
    LIBRARIES hltools_rad hltools_compile_parameters
    LABEL unit.rad)

add_hltools_unit(entity_csg
    PREFIX "entities.csg."
    SOURCES entities/csg/trigger_once.cpp
    LIBRARIES hltools_csg
    LABEL entities.csg)

# self contained checks keep their standalone entry points while ctest owns
# their invocation, arguments, labels, and working directory
add_hltools_check(foundation_check
    NAME integration.common.foundation
    SOURCES integration/common/foundation.cpp
    LIBRARIES hltools_math hltools_common
    LABELS integration common)

add_hltools_check(csg_brush_check
    NAME unit.csg.brush
    SOURCES unit/csg/brush.cpp
    LIBRARIES hltools_csg
    LABELS unit csg)

add_hltools_check(csg_brush_union_check
    NAME unit.csg.brush_union
    SOURCES unit/csg/brush_union.cpp
    LIBRARIES hltools_csg
    LABELS unit csg)

add_hltools_check(csg_stage_check
    NAME integration.csg.stage
    SOURCES integration/csg/stage.cpp
    LIBRARIES hltools_csg
    LABELS integration csg)

add_hltools_check(bsp_face_order_check
    NAME unit.bsp.face_order
    SOURCES unit/bsp/face_order.cpp
    LIBRARIES hltools_bsp
    LABELS unit bsp)

add_hltools_check(csg_wad_check
    NAME integration.csg.wad
    SOURCES integration/csg/wad.cpp
    LIBRARIES hltools_csg
    LABELS integration csg wad)

add_hltools_check(csg_bsp_output_check
    NAME integration.csg.bsp_output
    SOURCES integration/csg/bsp_output.cpp
    LIBRARIES hltools_csg
    LABELS integration csg bsp)

add_hltools_check(bsp_roundtrip
    NAME integration.format.bsp_roundtrip
    SOURCES integration/format/bsp_roundtrip.cpp
    LIBRARIES hltools_format
    ARGS
        "${HLTOOLS_TEST_SCRATCH_DIRECTORY}/csg_bsp_output_check/output.bsp"
        "${HLTOOLS_TEST_SCRATCH_DIRECTORY}/bsp_roundtrip.bsp"
    LABELS integration format bsp)
set_tests_properties(integration.format.bsp_roundtrip PROPERTIES
    DEPENDS integration.csg.bsp_output)

add_hltools_check(wad_tools_check
    NAME integration.tools.wad
    SOURCES integration/tools/wad.cpp
    LIBRARIES hltools_format
    ARGS "${HLTOOLS_TEST_SCRATCH_DIRECTORY}/wad_tools"
    LABELS integration tools wad)

add_hltools_check(lightmap_export_check
    NAME integration.tools.lightmap_export
    SOURCES integration/tools/lightmap_export.cpp
    LIBRARIES hltools_format
    ARGS "${HLTOOLS_TEST_SCRATCH_DIRECTORY}/lightmap_export"
    LABELS integration tools lightmap)

add_hltools_check(decompile_check
    NAME integration.tools.decompile
    SOURCES integration/tools/decompile.cpp
    LIBRARIES hltools_decompile hltools_csg
    ARGS "${HLTOOLS_TEST_SCRATCH_DIRECTORY}/decompile"
    LABELS integration tools decompile)

add_test(NAME integration.tools.bsp_port_help
    COMMAND $<TARGET_FILE:hltools> bsp port -h)
set_tests_properties(integration.tools.bsp_port_help PROPERTIES
    LABELS "integration;tools;port")

# these checks need the feature rich compiled bsp fixture they remain build
# targets until that fixture is generated as part of ctest rather than by hand
add_hltools_manual_check(bsp_inputs_check
    SOURCES integration/bsp/inputs.cpp
    LIBRARIES hltools_bsp)

if(HLTOOLS_GPU)
    add_hltools_manual_check(rad_gpu_trace_check
        SOURCES integration/rad/gpu_trace.cpp
        LIBRARIES hltools_rad hltools_rad_gpu)
    add_hltools_manual_check(rad_gpu_gather_check
        SOURCES integration/rad/gpu_gather.cpp
        LIBRARIES hltools_rad hltools_rad_gpu)
endif()
