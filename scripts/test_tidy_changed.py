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
#   Write a pytest suite for tidy_changed.py that needs no ROS 2, no build, no clang-tidy and no
#   git: small depfile texts shaped like CMake's, and a fake clang-tidy runner. Cover: depfile
#   parsing (continuations, empty, no colon); a changed .cpp selects only itself; a changed header
#   selects every TU that includes it, across packages; CMakeLists.txt and package.xml select the
#   whole package; a .msg selects the TUs that include generated headers; .clang-tidy (also
#   nested), docker/Dockerfile, the script itself and a changed TIDY_* variable select everything;
#   non-strict and not-compiled TUs are dropped; docs-only and empty changes select nothing.
#   Test files get the extra --checks, source files don't; a failing file fails the run and its
#   output is printed; main() exits 2 without a build. And: every family .clang-tidy enables,
#   except bugprone, is switched off in the Makefile's TIDY_TEST_CHECKS, so tests stay bugprone
#   only when someone adds a family.
"""Unit tests for the clang-tidy selection and runner of #144 (no ROS 2, no git)."""

import io
import os
import re

import tidy_changed as tc

STRICT = re.compile(r'^/ws/src/(pkg_a|pkg_b)/')
TESTS = re.compile(r'^/ws/src/[^/]+/test/')
REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


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


def sel(*changed, compiled=COMPILED, variables=()):
    selected, _ = tc.select(list(changed), INDEX, compiled, STRICT, variables=variables)
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
    assert sel('scripts/tidy_changed.py') == tc.ALL
    assert sel('docs/x.md', variables=['TIDY_TEST_CHECKS']) == tc.ALL


def test_other_changes_select_nothing():
    assert sel() == set()
    assert sel('docs/x.md', 'Makefile', 'docker/compose.yaml', 'scripts/x.py') == set()
    assert sel('src/pkg_a/include/pkg_a/unused.hpp') == set()
    assert sel('src/pkg_a/src/deleted.cpp') == set()


def test_reasons_name_the_changed_file():
    _, reasons = tc.select(['src/pkg_a/include/pkg_a/a.hpp', 'src/pkg_a/CMakeLists.txt'],
                           INDEX, COMPILED, STRICT)
    assert sorted(reasons[A_CPP]) == ['src/pkg_a/CMakeLists.txt (whole package)',
                                      'src/pkg_a/include/pkg_a/a.hpp']
    _, reasons = tc.select(['Dockerfile', 'docker/Dockerfile'], INDEX, COMPILED, STRICT)
    assert reasons == {tc.ALL: ['docker/Dockerfile changed']}


def test_make_variable_and_changed_variables():
    base = 'X := 1\nTIDY_STRICT_FILES := ^/ws/src/(a|b)/\nTIDY_TEST_CHECKS := -misc-*\n'
    assert tc.make_variable(base, 'TIDY_STRICT_FILES') == '^/ws/src/(a|b)/'
    assert tc.make_variable(base, 'TIDY_TEST_FILES') is None
    assert tc.changed_variables(base, base + 'Y := 2\n') == []
    head = base.replace('-misc-*', '-misc-*,-cert-*') + 'TIDY_TEST_FILES := /test/\n'
    assert tc.changed_variables(base, head) == ['TIDY_TEST_FILES', 'TIDY_TEST_CHECKS']
    # A Makefile that doesn't exist at the base (new repo, or git show failed) is a change.
    assert tc.changed_variables('', base) == ['TIDY_STRICT_FILES', 'TIDY_TEST_CHECKS']


def test_all_strict_drops_other_packages():
    assert tc.all_strict(COMPILED, STRICT) == {A_CPP, A_TEST, B_CPP}
    assert tc.all_strict(set(), STRICT) == set()


def test_test_files_get_the_test_checks_and_sources_do_not():
    assert tc.tidy_command(A_TEST, 'build', TESTS, '-misc-*') == [
        'clang-tidy', '-p', 'build', '--quiet', '--warnings-as-errors=*', '--checks=-misc-*',
        A_TEST]
    assert '--checks=-misc-*' not in tc.tidy_command(A_CPP, 'build', TESTS, '-misc-*')
    # A file called test_x.cpp outside test/ is source, not a test.
    assert not TESTS.search('/ws/src/pkg_a/src/test_x.cpp')


def fake_runner(failing):
    calls = []

    def runner(command):
        calls.append(command)
        if command[-1] in failing:
            return 1, 2.5, command[-1] + ':1:1: error: planted [x,-warnings-as-errors]\n'
        return 0, 1.0, ''
    return runner, calls


def test_run_all_passes_and_prints_one_line_per_file():
    runner, calls = fake_runner(set())
    out = io.StringIO()
    results = tc.run_all({A_CPP, A_TEST}, 'build', TESTS, '-misc-*', 2, runner, out)
    assert {tu: r[0] for tu, r in results.items()} == {A_CPP: 0, A_TEST: 0}
    assert len(calls) == 2
    lines = out.getvalue().splitlines()
    assert len(lines) == 2 and 'ok' in lines[0] and 'src' in lines[0] and 'test' in lines[1]


def test_run_all_failure_is_reported_with_its_output():
    runner, _ = fake_runner({A_TEST})
    out = io.StringIO()
    results = tc.run_all({A_CPP, A_TEST}, 'build', TESTS, '-misc-*', 1, runner, out)
    assert results[A_TEST][0] == 1 and results[A_CPP][0] == 0
    assert 'FAIL' in out.getvalue() and 'planted' in out.getvalue()


def test_run_all_with_nothing_to_check():
    runner, calls = fake_runner(set())
    assert tc.run_all(set(), 'build', TESTS, '', 0, runner, io.StringIO()) == {}
    assert calls == []


def test_report_full_changed_and_failed():
    results = {A_CPP: (0, 3.0, ''), A_TEST: (1, 9.0, 'x')}
    text = tc.report(None, tc.ALL, {}, results, TESTS, 12.0)
    assert '**Full run** (`make tidy-strict`)' in text
    assert '2 files (1 src, 1 test with bugprone only), 1 failed, 12 s' in text
    assert text.index(A_TEST) < text.index(A_CPP)  # slowest first
    assert '**FAIL**' in text
    text = tc.report(['docker/Dockerfile'], tc.ALL, {tc.ALL: ['docker/Dockerfile changed']},
                     {}, TESTS)
    assert '**Full run**: docker/Dockerfile changed' in text
    text = tc.report(['a.hpp'], {A_CPP}, {A_CPP: ['a.hpp']}, {A_CPP: (0, 1.0, '')}, TESTS)
    assert 'Strict TUs they can affect: **1**' in text and '(a.hpp)' in text


def test_main_without_a_build_exits_2(tmp_path, capsys):
    code = tc.main(['--all', '--strict', 'x', '--test-files', 'y', '--test-checks=-z',
                    '--build-dir', str(tmp_path)])
    assert code == 2
    assert 'run make build first' in capsys.readouterr().err


def test_main_all_runs_every_strict_tu_and_fails_on_a_warning(tmp_path, monkeypatch):
    (tmp_path / 'compile_commands.json').write_text(
        '[' + ','.join('{"file": "%s"}' % tu for tu in sorted(COMPILED)) + ']')
    runner, calls = fake_runner({B_CPP})
    monkeypatch.setattr(tc, 'run_one', runner)
    summary = tmp_path / 'log' / 'summary.md'
    monkeypatch.setenv('GITHUB_STEP_SUMMARY', str(summary))
    argv = ['--all', '--strict', STRICT.pattern, '--test-files', TESTS.pattern,
            '--test-checks=-misc-*', '--build-dir', str(tmp_path), '--jobs', '2']
    assert tc.main(argv) == 1
    assert sorted(c[-1] for c in calls) == sorted([A_CPP, A_TEST, B_CPP])
    assert '1 failed' in summary.read_text()
    assert tc.main(argv + ['--dry-run']) == 0


def test_tests_get_only_bugprone_whatever_clang_tidy_enables():
    with open(os.path.join(REPO, '.clang-tidy'), encoding='utf-8') as f:
        config = f.read()
    with open(os.path.join(REPO, 'Makefile'), encoding='utf-8') as f:
        disabled = tc.make_variable(f.read(), 'TIDY_TEST_CHECKS').split(',')
    checks = config.split('Checks:', 1)[1].split('\n\n', 1)[0]
    families = {c.strip(' >,\n') for c in checks.split(',')}
    enabled = {c for c in families if c.endswith('-*') and not c.startswith('-')}
    assert 'bugprone-*' in enabled
    assert enabled - {'bugprone-*'} == {c[1:] for c in disabled}
