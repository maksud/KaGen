################################################################################
# tests/CMakeLists.txt
#
# Part of tlx - http://panthema.net/tlx
#
# Copyright (C) 2015-2017 Timo Bingmann <tb@panthema.net>
#
# All rights reserved. Published under the Boost Software License, Version 1.0
################################################################################

# macro for building test target programs with correct libraries
macro(tlx_build_target TARGETNAME)

  add_executable(tlx_${TARGETNAME} ${ARGN})
  target_link_libraries(tlx_${TARGETNAME} tlx)

endmacro(tlx_build_target)

# macro for building test programs, without main() in gtest_main
macro(tlx_build_plain PROGNAME)

  string(REPLACE "/" "_" TESTNAME "${PROGNAME}") # replace slashes

  tlx_build_target(${TESTNAME} ${PROGNAME}.cpp ${ARGN})

endmacro(tlx_build_plain)

# macro for building test programs, adding gtest runner in gtest_main
macro(tlx_build_only PROGNAME)

  # append gtest runner program.
  tlx_build_plain(${PROGNAME} ${ARGN})

endmacro(tlx_build_only)

# macro for registering test programs: maybe prepend valgrind
macro(tlx_test_only TESTNAME)

  set(TARGETNAME ${TESTNAME} ${ARGN})
  string(REPLACE "/" "_" TARGETNAME "${TARGETNAME}") # replace slashes
  string(REPLACE ";" "_" TARGETNAME "${TARGETNAME}") # stringify list

  add_test(
    NAME tlx_${TARGETNAME}
    COMMAND tlx_${TESTNAME} ${ARGN})

endmacro(tlx_test_only)

# macro for building and running test programs
macro(tlx_build_test PROGNAME)

  tlx_build_only(${PROGNAME})

  string(REPLACE "/" "_" TESTNAME "${PROGNAME}") # replace slashes
  tlx_test_only(${TESTNAME})

endmacro(tlx_build_test)

### list of tests in subdirectories

tlx_build_only(btree_speedtest)
tlx_build_only(cmdline_parser_example)

tlx_build_test(backtrace_test)
tlx_build_test(btree_test)
tlx_build_test(cmdline_parser_test)
tlx_build_test(counting_ptr_test)
tlx_build_test(delegate_test)
tlx_build_test(die_test)
if(TLX_CXX_HAS_CXX14)
  tlx_build_test(function_chain_test)
  tlx_build_test(function_stack_test)
endif()
tlx_build_test(loser_tree_test)
tlx_build_test(lru_cache_test)
tlx_build_test(math_test)
tlx_build_test(meta_has_member_test)
tlx_build_test(meta_log2_test)
if(TLX_CXX_HAS_CXX14)
  tlx_build_test(meta_call_for_test)
  tlx_build_test(meta_vmap_for_test)
endif()
tlx_build_test(ring_buffer_test)
tlx_build_test(simple_vector_test)
tlx_build_test(siphash_test)
tlx_build_test(stack_allocator_test)
tlx_build_test(string_test)
tlx_build_test(thread_pool_test)

# disable -Wshadow on source with FunctionStack or FunctionChain
if(NOT MSVC)
  set_source_files_properties(
    function_chain_test.cpp  function_stack_test.cpp
    PROPERTIES COMPILE_FLAGS "-Wno-shadow")
endif()

################################################################################
