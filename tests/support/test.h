#pragma once

#define DOCTEST_CONFIG_NO_SHORT_MACRO_NAMES
#include <doctest/doctest.h>

// keep the test vocabulary small and independent of the underlying framework
// these aliases belong only in test translation units
#define suite(...) DOCTEST_TEST_SUITE(__VA_ARGS__)
#define test(...) DOCTEST_TEST_CASE(__VA_ARGS__)
#define expect(...) DOCTEST_CHECK(__VA_ARGS__)
#define expect_false(...) DOCTEST_CHECK_FALSE(__VA_ARGS__)
#define require(...) DOCTEST_REQUIRE(__VA_ARGS__)
#define require_false(...) DOCTEST_REQUIRE_FALSE(__VA_ARGS__)
#define capture(...) DOCTEST_CAPTURE(__VA_ARGS__)
