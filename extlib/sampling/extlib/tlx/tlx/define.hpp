/*******************************************************************************
 * tlx/define.hpp
 *
 * Part of tlx - http://panthema.net/tlx
 *
 * Copyright (C) 2017 Timo Bingmann <tb@panthema.net>
 *
 * All rights reserved. Published under the Boost Software License, Version 1.0
 ******************************************************************************/

#ifndef TLX_DEFINE_HEADER
#define TLX_DEFINE_HEADER

/*[[[perl
print "#include <$_>\n" foreach sort glob("tlx/define/"."*.hpp");
]]]*/
#include <tlx/define/attribute_always_inline.hpp>
#include <tlx/define/attribute_fallthrough.hpp>
#include <tlx/define/attribute_format_printf.hpp>
#include <tlx/define/attribute_packed.hpp>
#include <tlx/define/attribute_warn_unused_result.hpp>
#include <tlx/define/likely.hpp>
// [[[end]]]

#endif // !TLX_DEFINE_HEADER

/******************************************************************************/
