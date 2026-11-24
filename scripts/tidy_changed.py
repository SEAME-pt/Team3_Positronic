#!/usr/bin/env python3
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
#   Issue #144: on a PR, run clang-tidy only on the translation units the PR can affect, and
#   everything on a push to dev. Read the changed files from git (merge base..working tree), and
#   the CMake depfiles build/<pkg>/CMakeFiles/<target>.dir/**/*.o.d, which list every header each
#   TU includes, to map a changed header to the TUs that include it. A changed CMakeLists.txt or
#   package.xml selects the whole package; a changed .msg/.srv/.action selects every TU that
#   includes that package's generated headers; a changed .clang-tidy, docker/Dockerfile, this
#   script or one of the TIDY_* variables in the Makefile selects everything. Keep only TUs in
#   compile_commands.json that match TIDY_STRICT_FILES. Then run clang-tidy on them in parallel,
#   every warning an error; test files (TIDY_TEST_FILES) get only the bugprone checks
#   (TIDY_TEST_CHECKS, ADR 0007). Log each file's time, and write a Markdown report to
#   $GITHUB_STEP_SUMMARY when it is set. Pure functions where possible, stdlib only, unit-tested.
#   Later: a --soft mode for make tidy, on all of src/ with the same test-file rule: warnings are
#   printed, not errors (ADR 0004), so only a file clang-tidy can't check fails the run.
#   And a summary at the end of every run, like make test's (#148, reuse its Style): one line
#   per package with files, warnings, the checks that fired most and time, then a verdict line.
#   Count a warning in the package of the file it points at, once, even when a header's warning
#   shows up in every file that includes it.
"""clang-tidy, warnings as errors, on the strict TUs a change can affect (issue #144)."""

#   make tidy-strict    python3 scripts/tidy_changed.py --all <TIDY_* options>
#   make tidy-changed   python3 scripts/tidy_changed.py --base origin/dev <TIDY_* options>
#   make tidy           python3 scripts/tidy_changed.py --all --soft --strict '^/ws/src/' ...
# --dry-run prints the selection without running clang-tidy.
# git runs inside the dev container, so --base doesn't work in a git worktree (its .git file
# points outside the mounted repo): use make tidy-strict there.

import argparse
import collections
from concurrent.futures import ThreadPoolExecutor
import json
import os
import re
import subprocess
import sys
import time

from summarize_tests import Style, use_colour

ALL = 'ALL'
FULL_RUN_FILES = ('docker/Dockerfile', 'scripts/tidy_changed.py')
FULL_RUN_BASENAMES = ('.clang-tidy',)
PACKAGE_FILES = ('CMakeLists.txt', 'package.xml')
INTERFACE_SUFFIXES = ('.msg', '.srv', '.action', '.idl')
MAKE_VARIABLES = ('TIDY_STRICT_FILES', 'TIDY_TEST_FILES', 'TIDY_TEST_CHECKS')
# path:line:col: warning: text [check,check] (strict: error: ... [check,-warnings-as-errors])
DIAGNOSTIC = re.compile(r'^(\S+?):(\d+):(\d+): (?:warning|error): .*?(?: \[([^\]]+)\])?$',
                        re.MULTILINE)
TOP_CHECKS = 3


def parse_depfile(text):
    """Prerequisites of a Make depfile, in order: the first one is the source file."""
    _, sep, rest = text.partition(': ')
    if not sep:
        return []
    rest = rest.replace('\\\n', ' ')
    return [token for token in rest.split() if token != '\\']


def find_depfiles(build_dir):
    """Every build/<pkg>/CMakeFiles/**/*.o.d below build_dir."""
    found = []
    for pkg in sorted(os.listdir(build_dir)):
        cmake_files = os.path.join(build_dir, pkg, 'CMakeFiles')
        for dirpath, _, filenames in os.walk(cmake_files):
            found.extend(os.path.join(dirpath, n) for n in filenames if n.endswith('.o.d'))
    return sorted(found)


def build_index(depfile_texts):
    """Map each TU (the depfile's first prerequisite) to the set of files it depends on."""
    index = {}
    for text in depfile_texts:
        prerequisites = parse_depfile(text)
        if prerequisites:
            index.setdefault(prerequisites[0], set()).update(prerequisites)
    return index


def make_variable(makefile_text, name):
    """Return the value of a ':=' variable in the Makefile, or None."""
    match = re.search(r'^' + name + r'\s*:?=\s*(.*)$', makefile_text, re.MULTILINE)
    return match.group(1).strip() if match else None


def changed_variables(base_makefile, head_makefile):
    """List the TIDY_* variables whose value differs between the two Makefiles."""
    return [name for name in MAKE_VARIABLES
            if make_variable(base_makefile, name) != make_variable(head_makefile, name)]


def package_of(path):
    """'src/<pkg>/...' -> '<pkg>', anything else -> None."""
    parts = path.split('/')
    return parts[1] if len(parts) > 2 and parts[0] == 'src' else None


def full_run_reason(changed, variables=()):
    """Why the change needs the full run, or None."""
    for path in changed:
        if path in FULL_RUN_FILES or os.path.basename(path) in FULL_RUN_BASENAMES:
            return path + ' changed'
    if variables:
        return ', '.join(variables) + ' changed in the Makefile'
    return None


def select(changed, index, compiled, strict, root='/ws', variables=()):
    """
    Return (ALL, {ALL: [reason]}) or (set of TUs, {TU: [reasons]}).

    changed: repo-relative paths; index: build_index(); compiled: TUs in compile_commands.json;
    strict: compiled regex on the TU's absolute path; variables: changed_variables().
    """
    reason = full_run_reason(changed, variables)
    if reason:
        return ALL, {ALL: [reason]}
    reasons = {}

    def add(tu, why):
        if tu in compiled and strict.search(tu):
            reasons.setdefault(tu, []).append(why)

    for path in changed:
        absolute = root + '/' + path
        pkg = package_of(path)
        name = os.path.basename(path)
        if pkg and name in PACKAGE_FILES:
            for tu in index:
                if tu.startswith(root + '/src/' + pkg + '/'):
                    add(tu, path + ' (whole package)')
        if pkg and (name.endswith(INTERFACE_SUFFIXES) or name in PACKAGE_FILES):
            generated = root + '/build/' + pkg + '/'
            for tu, deps in index.items():
                if any(dep.startswith(generated) for dep in deps):
                    add(tu, path + ' (generated headers)')
        for tu, deps in index.items():
            if absolute in deps:
                add(tu, path)
    return set(reasons), reasons


def all_strict(compiled, strict):
    """Every compiled TU that matches TIDY_STRICT_FILES."""
    return {tu for tu in compiled if strict.search(tu)}


def tidy_command(tu, build_dir, test_files, test_checks, soft=False):
    """Build the clang-tidy call for one TU: warnings are errors unless soft, fewer on tests."""
    command = ['clang-tidy', '-p', build_dir, '--quiet']
    if not soft:
        command.append('--warnings-as-errors=*')
    if test_files.search(tu):
        command.append('--checks=' + test_checks)
    return command + [tu]


def run_one(command):
    """Run one clang-tidy call: (exit code, seconds, output)."""
    start = time.monotonic()
    done = subprocess.run(command, capture_output=True, text=True, check=False)
    output = done.stdout
    if done.returncode != 0:
        output += done.stderr
    return done.returncode, time.monotonic() - start, output


def verdict(code, output):
    """'FAIL' when clang-tidy failed, 'warn' when it passed with output (soft), else 'ok'."""
    if code != 0:
        return 'FAIL'
    return 'warn' if output.strip() else 'ok'


def run_all(tus, build_dir, test_files, test_checks, jobs, runner=None, out=None, soft=False):
    """Run clang-tidy on every TU, jobs at a time; print each in order. Return {TU: result}."""
    runner = runner or run_one
    out = out or sys.stdout
    results = {}
    ordered = sorted(tus)
    with ThreadPoolExecutor(max_workers=max(1, jobs)) as pool:
        futures = {tu: pool.submit(runner, tidy_command(tu, build_dir, test_files, test_checks,
                                                        soft))
                   for tu in ordered}
        for n, tu in enumerate(ordered, start=1):
            code, seconds, output = futures[tu].result()
            results[tu] = (code, seconds, output)
            kind = 'test' if test_files.search(tu) else 'src'
            line = '[%d/%d] %-4s %6.1f s  %-4s %s' % (n, len(ordered), verdict(code, output),
                                                      seconds, kind, tu)
            out.write(line + '\n')
            if output.strip():
                out.write(output.rstrip() + '\n')
            out.flush()
    return results


def warnings_in(output):
    """Return {diagnostic line: (path, [checks])} for each warning or error clang-tidy printed."""
    found = {}
    for match in DIAGNOSTIC.finditer(output):
        checks = [c for c in (match.group(4) or 'clang-diagnostic-error').split(',')
                  if c != '-warnings-as-errors']
        found[match.group(0)] = (match.group(1), checks)
    return found


def summarize(results, test_files, root='/ws'):
    """Per package: {pkg: {'src', 'test', 'failed', 'seconds', 'warnings', 'checks'}}.

    Files and time count in the package of the TU; a warning counts once (a header's warning
    is printed for every TU that includes it), in the package of the file it points at.
    """
    def entry(pkg):
        return packages.setdefault(pkg, {'src': 0, 'test': 0, 'failed': 0, 'seconds': 0.0,
                                         'warnings': 0, 'checks': collections.Counter()})

    packages, seen = {}, set()
    for tu, (code, seconds, output) in sorted(results.items()):
        pkg = entry(package_of(tu[len(root) + 1:]) or '(other)')
        pkg['test' if test_files.search(tu) else 'src'] += 1
        pkg['failed'] += code != 0
        pkg['seconds'] += seconds
        for key, (path, checks) in warnings_in(output).items():
            if key in seen:
                continue
            seen.add(key)
            where = entry(package_of(path[len(root) + 1:]) if path.startswith(root + '/')
                          else '(other)')
            where['warnings'] += 1
            where['checks'].update(checks)
    return packages


def summary_text(packages, soft, wall, style):
    """Build the end of the run: a line per package, then the verdict. Return (text, passed)."""
    width = max([len(p) for p in packages] + [7])
    lines = ['', style.bold('clang-tidy summary'), '']
    for name, p in sorted(packages.items()):
        files = p['src'] + p['test']
        mark = (style.red('✘') if p['failed'] or (p['warnings'] and not soft)
                else style.yellow('!') if p['warnings'] else style.green('✔'))
        counts = '%3d files (%d src, %d test)' % (files, p['src'], p['test'])
        warned = '%3d warnings' % p['warnings']
        if p['warnings']:
            warned = (style.yellow if soft else style.red)(warned)
        top = ', '.join('%s ×%d' % c for c in p['checks'].most_common(TOP_CHECKS))
        lines.append(('  %s  %-*s  %-22s  %s  %6.0f s CPU  %s' % (
            mark, width, name, counts, warned, p['seconds'], style.dim(top))).rstrip())
    files = sum(p['src'] + p['test'] for p in packages.values())
    warnings = sum(p['warnings'] for p in packages.values())
    failed = sum(p['failed'] for p in packages.values())
    why = '%d files, %d warnings, %d failed, %.0f s wall' % (files, warnings, failed, wall)
    lines.append('')
    if failed or (warnings and not soft):
        lines.append(style.bold(style.red('✘ FAILED: ' + why)))
        return '\n'.join(lines) + '\n', False
    if warnings:
        why += ", warnings don't fail make tidy"
        lines.append(style.bold(style.yellow('! WARNINGS: ' + why)))
    else:
        lines.append(style.bold(style.green('✔ PASSED: ' + why)))
    return '\n'.join(lines) + '\n', True


def report(changed, selected, reasons, results, test_files, wall=None):
    """Markdown: why these TUs, then the time and verdict of each."""
    lines = ['### clang-tidy', '']
    if changed is None:
        lines.append('**Full run** (`make tidy-strict`)')
    elif selected == ALL:
        lines.append('Changed files: %d. **Full run**: %s' % (len(changed), reasons[ALL][0]))
    else:
        lines.append('Changed files: %d. Strict TUs they can affect: **%d**'
                     % (len(changed), len(selected)))
    failed = sorted(tu for tu, result in results.items() if result[0] != 0)
    tests = sum(1 for tu in results if test_files.search(tu))
    lines.append('')
    lines.append('%d files (%d src, %d test with bugprone only), %d failed%s'
                 % (len(results), len(results) - tests, tests, len(failed),
                    '' if wall is None else ', %.0f s' % wall))
    if results:
        lines += ['', '| Package | Files | Warnings | Most frequent | Time |',
                  '|---|--:|--:|---|--:|']
        for name, p in sorted(summarize(results, test_files).items()):
            top = ', '.join('`%s` ×%d' % c for c in p['checks'].most_common(TOP_CHECKS))
            lines.append('| %s | %d | %d | %s | %.1f s |' % (
                name, p['src'] + p['test'], p['warnings'], top, p['seconds']))
        lines += ['', '| File | Checks | Time | Result |', '|---|---|---:|---|']
        for tu in sorted(results, key=lambda t: -results[t][1]):
            code, seconds, _ = results[tu]
            why = '' if selected == ALL or changed is None else (
                ' (' + ', '.join(sorted(set(reasons[tu]))) + ')')
            lines.append('| `%s`%s | %s | %.1f s | %s |' % (
                tu, why, 'bugprone' if test_files.search(tu) else 'all', seconds,
                'ok' if code == 0 else '**FAIL**'))
    return '\n'.join(lines) + '\n'


def git(*args):
    """Stdout of a git command in the repo; raises on failure."""
    # safe.directory: Docker Desktop shows the mounted repo as owned by root, and git refuses
    # to read a repo owned by someone else.
    return subprocess.run(['git', '-c', 'safe.directory=' + os.getcwd(), *args], check=True,
                          capture_output=True, text=True).stdout


def git_show(ref, path):
    """Return a file at a git ref, or '' when it doesn't exist there."""
    try:
        return git('show', ref + ':' + path)
    except subprocess.CalledProcessError:
        return ''


def changes_since(base_ref):
    """Files changed from the merge base with base_ref to the working tree, and TIDY_* changes."""
    base = git('merge-base', base_ref, 'HEAD').strip()
    changed = [p for p in git('diff', '--name-only', '--no-renames', base).splitlines() if p]
    with open('Makefile', encoding='utf-8') as f:
        head_makefile = f.read()
    return changed, changed_variables(git_show(base, 'Makefile'), head_makefile)


def main(argv=None):
    """Select, run and report; 0 when every file passed, 1 on a warning, 2 on a usage error.

    With --soft a warning doesn't fail: 1 only when clang-tidy can't check a file.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    what = parser.add_mutually_exclusive_group(required=True)
    what.add_argument('--all', action='store_true', help='every strict TU (make tidy-strict)')
    what.add_argument('--base', help='only the TUs changed since the merge base with this ref')
    parser.add_argument('--strict', required=True, help='TIDY_STRICT_FILES')
    parser.add_argument('--test-files', required=True, help='TIDY_TEST_FILES')
    parser.add_argument('--test-checks', required=True, help='TIDY_TEST_CHECKS')
    parser.add_argument('--build-dir', default='build')
    parser.add_argument('--root', default='/ws', help='the repo path inside the depfiles')
    parser.add_argument('--jobs', type=int, default=os.cpu_count())
    parser.add_argument('--dry-run', action='store_true', help='print the selection only')
    parser.add_argument('--soft', action='store_true',
                        help='warnings are printed, not errors (make tidy, ADR 0004)')
    args = parser.parse_args(argv)
    start = time.monotonic()

    database = os.path.join(args.build_dir, 'compile_commands.json')
    if not os.path.exists(database):
        print('No %s: run make build first' % database, file=sys.stderr)
        return 2
    with open(database, encoding='utf-8') as f:
        compiled = {entry['file'] for entry in json.load(f)}
    strict = re.compile(args.strict)
    test_files = re.compile(args.test_files)

    changed, reasons = None, {}
    if args.all:
        selected = ALL
    else:
        try:
            changed, variables = changes_since(args.base)
        except subprocess.CalledProcessError as error:
            print('git failed: %s\n%s' % (' '.join(error.cmd), error.stderr), file=sys.stderr)
            print('(In a git worktree, git does not work in the dev container: '
                  'use make tidy-strict.)', file=sys.stderr)
            return 2
        texts = []
        for depfile in find_depfiles(args.build_dir):
            with open(depfile, encoding='utf-8') as f:
                texts.append(f.read())
        selected, reasons = select(changed, build_index(texts), compiled, strict, args.root,
                                   variables)
    tus = all_strict(compiled, strict) if selected == ALL else selected

    if changed is not None:
        print('%d changed files since %s' % (len(changed), args.base))
        if selected == ALL:
            print('Full run: ' + reasons[ALL][0])
        for tu in sorted(tus if selected != ALL else []):
            print('  %s <- %s' % (tu, ', '.join(sorted(set(reasons[tu])))))
    print('%d files to check' % len(tus))
    if args.dry_run:
        return 0

    results = run_all(tus, args.build_dir, test_files, args.test_checks, args.jobs,
                      soft=args.soft)
    text = report(changed, selected, reasons, results, test_files, time.monotonic() - start)
    if results:
        summary, _ = summary_text(summarize(results, test_files), args.soft,
                                  time.monotonic() - start, Style(use_colour(sys.stdout)))
        print(summary, end='')
    failed = sum(1 for result in results.values() if result[0] != 0)
    summary = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary:
        os.makedirs(os.path.dirname(summary) or '.', exist_ok=True)
        with open(summary, 'a', encoding='utf-8') as f:
            f.write(text)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
