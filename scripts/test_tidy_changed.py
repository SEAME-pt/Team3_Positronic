# Copyright 2026 Positronic
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# AI-generated (Claude, Anthropic).
# Prompt:
#   Write a pytest suite for tidy_changed.py that needs no ROS 2, no build and no git: small
#   depfile texts shaped like CMake's. Cover: depfile parsing (continuations, empty, no colon);
#   a changed .cpp selects only itself; a changed header selects every TU that includes it,
#   across packages; CMakeLists.txt and package.xml select the whole package; a .msg selects
#   the TUs that include generated headers; .clang-tidy (also nested), docker/Dockerfile and a
#   changed TIDY_STRICT_FILES select everything; non-strict and not-compiled TUs are dropped;
#   docs-only and empty changes select nothing; the regex matches exactly the TUs.
"""Unit tests for the changed-TU selection of #144 experiment E4 (no ROS 2, no git)."""

import re

import tidy_changed as tc

STRICT = re.compile(r'^/ws/src/(pkg_a|pkg_b)/')


def depfile(target, source, *headers):
    lines = [target + ': \\', ' ' + source + ' \\']
    lines += [' ' + h + ' \\' for h in headers[:-1]]
    if headers:
        lines.append(' ' + headers[-1])
    return '\n'.join(lines) + '\n'


A_HPP = '/ws/src/pkg_a/include/pkg_a/a.hpp'
B_HPP = '/ws/src/pkg_b/include/pkg_b/b.hpp'
GEN = '/ws/build/pkg_msgs/rosidl_generator_cpp/pkg_msgs/msg/status.hpp'
A_CPP = '/ws/src/pkg_a/src/a.cpp'
A_TEST = '/ws/src/pkg_a/test/test_a.cpp'
B_CPP = '/ws/src/pkg_b/src/b.cpp'
C_CPP = '/ws/src/pkg_c/src/c.cpp'  # not strict
DEPFILES = [
    depfile('CMakeFiles/a.dir/src/a.cpp.o', A_CPP, A_HPP, '/opt/ros/jazzy/include/x.hpp'),
    depfile('CMakeFiles/test_a.dir/test/test_a.cpp.o', A_TEST, A_HPP),
    depfile('CMakeFiles/b.dir/src/b.cpp.o', B_CPP, B_HPP, A_HPP, GEN),
    depfile('CMakeFiles/c.dir/src/c.cpp.o', C_CPP, A_HPP),
]
INDEX = tc.build_index(DEPFILES)
COMPILED = {A_CPP, A_TEST, B_CPP, C_CPP}


def sel(*changed, compiled=COMPILED, strict_changed=False):
    selected, _ = tc.select(list(changed), INDEX, compiled, STRICT, strict_changed=strict_changed)
    return selected


def test_parse_depfile_keeps_order_and_joins_continuations():
    assert tc.parse_depfile(DEPFILES[0]) == [A_CPP, A_HPP, '/opt/ros/jazzy/include/x.hpp']


def test_parse_depfile_empty_or_without_colon():
    assert tc.parse_depfile('') == []
    assert tc.parse_depfile('no prerequisites here') == []


def test_build_index_keys_are_sources():
    assert set(INDEX) == {A_CPP, A_TEST, B_CPP, C_CPP}
    assert A_HPP in INDEX[B_CPP]


def test_changed_cpp_selects_only_itself():
    assert sel('src/pkg_a/src/a.cpp') == {A_CPP}


def test_changed_header_selects_includers_across_packages_but_not_non_strict():
    assert sel('src/pkg_a/include/pkg_a/a.hpp') == {A_CPP, A_TEST, B_CPP}


def test_header_included_once():
    assert sel('src/pkg_b/include/pkg_b/b.hpp') == {B_CPP}


def test_not_compiled_tu_is_dropped():
    assert sel('src/pkg_a/include/pkg_a/a.hpp', compiled={A_CPP}) == {A_CPP}


def test_cmakelists_and_package_xml_select_whole_package():
    assert sel('src/pkg_a/CMakeLists.txt') == {A_CPP, A_TEST}
    assert sel('src/pkg_a/package.xml') == {A_CPP, A_TEST}


def test_msg_selects_tus_with_generated_headers():
    assert sel('src/pkg_msgs/msg/Status.msg') == {B_CPP}
    assert sel('src/pkg_msgs/action/Go.action') == {B_CPP}


def test_full_run_triggers():
    assert sel('.clang-tidy') == tc.ALL
    assert sel('src/pkg_a/test/.clang-tidy') == tc.ALL
    assert sel('docker/Dockerfile', 'docs/x.md') == tc.ALL
    assert sel('docs/x.md', strict_changed=True) == tc.ALL


def test_other_changes_select_nothing():
    assert sel() == set()
    assert sel('docs/x.md', 'Makefile', 'docker/compose.yaml', 'scripts/x.py') == set()
    assert sel('src/pkg_a/include/pkg_a/unused.hpp') == set()


def test_reasons_name_the_changed_file():
    _, reasons = tc.select(['src/pkg_a/include/pkg_a/a.hpp', 'src/pkg_a/CMakeLists.txt'],
                           INDEX, COMPILED, STRICT)
    assert sorted(reasons[A_CPP]) == ['src/pkg_a/CMakeLists.txt (whole package)',
                                      'src/pkg_a/include/pkg_a/a.hpp']


def test_strict_regex_from_makefile():
    text = 'X := 1\nTIDY_STRICT_FILES := ^/ws/src/(a|b)/\n'
    assert tc.strict_regex(text) == '^/ws/src/(a|b)/'
    assert tc.strict_regex('X := 1\n') is None


def test_regex_matches_exactly_the_selected_tus():
    pattern = re.compile(tc.to_regex({A_CPP, B_CPP}))
    assert pattern.search(A_CPP) and pattern.search(B_CPP)
    assert not pattern.search(A_TEST)
    assert not pattern.search(A_CPP + '.bak')
    assert not pattern.search('/ws/src/pkg_a/src/aXcpp')


def test_report_full_and_selected():
    assert '**Full run**: docker/Dockerfile changed' in tc.report(
        ['docker/Dockerfile'], tc.ALL, {tc.ALL: ['docker/Dockerfile changed']})
    text = tc.report(['a'], {A_CPP}, {A_CPP: ['a']}, 0.5)
    assert 'Selected TUs: **1**' in text and 'Selection took 0.50 s' in text
