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
#   Experiment E4 of #144 (plan B1 + B2): on a PR, run clang-tidy only on the translation units
#   the PR can affect. Read the changed files from git (base..head), and the CMake depfiles
#   build/<pkg>/CMakeFiles/<target>.dir/**/*.o.d, which list every header each TU includes, to
#   map a changed header to the TUs that include it. A changed CMakeLists.txt or package.xml
#   selects the whole package; a changed .msg/.srv/.action selects every TU that includes that
#   package's generated headers; a changed .clang-tidy, docker/Dockerfile or TIDY_STRICT_FILES
#   line in the Makefile selects everything. Keep only TUs in compile_commands.json that match
#   TIDY_STRICT_FILES. Pure functions, stdlib only, so it runs on the runner and is unit-tested.
"""Select the clang-tidy translation units a change can affect (issue #144, experiment E4)."""

#   python3 scripts/tidy_changed.py --base HEAD^1 --head HEAD
#       prints ALL, or the selected TUs one per line (nothing when no TU is affected)
#   ... --format regex      ALL, an anchored regex for run-clang-tidy, or nothing
#   ... --summary FILE      also append a Markdown report (for $GITHUB_STEP_SUMMARY)
# The report (why each TU was selected) goes to stderr.

import argparse
import json
import os
import re
import subprocess
import sys
import time

ALL = 'ALL'
FULL_RUN_FILES = ('docker/Dockerfile',)
FULL_RUN_BASENAMES = ('.clang-tidy',)
PACKAGE_FILES = ('CMakeLists.txt', 'package.xml')
INTERFACE_SUFFIXES = ('.msg', '.srv', '.action', '.idl')
STRICT_VARIABLE = 'TIDY_STRICT_FILES'


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


def strict_regex(makefile_text):
    """Return the value of TIDY_STRICT_FILES in the Makefile, or None."""
    match = re.search(r'^' + STRICT_VARIABLE + r'\s*:?=\s*(.*)$', makefile_text, re.MULTILINE)
    return match.group(1).strip() if match else None


def package_of(path):
    """'src/<pkg>/...' -> '<pkg>', anything else -> None."""
    parts = path.split('/')
    return parts[1] if len(parts) > 2 and parts[0] == 'src' else None


def full_run_reason(changed, strict_changed):
    """Why the change needs the full run, or None."""
    for path in changed:
        if path in FULL_RUN_FILES or os.path.basename(path) in FULL_RUN_BASENAMES:
            return path + ' changed'
    if strict_changed:
        return STRICT_VARIABLE + ' changed in the Makefile'
    return None


def select(changed, index, compiled, strict, root='/ws', strict_changed=False):
    """
    Return (ALL, {}) or (set of TUs, {TU: [reasons]}).

    changed: repo-relative paths; index: build_index(); compiled: TUs in compile_commands.json;
    strict: compiled regex on the TU's absolute path.
    """
    reason = full_run_reason(changed, strict_changed)
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


def to_regex(tus):
    """Anchored regex matching exactly these TUs (run-clang-tidy's positional file filter)."""
    return '^(' + '|'.join(re.escape(tu) for tu in sorted(tus)) + ')$'


def report(changed, selected, reasons, elapsed=None):
    """Markdown: changed files, then each selected TU and why."""
    lines = ['### clang-tidy: changed TUs (#144 E4)', '']
    lines.append('Changed files: %d' % len(changed))
    if selected == ALL:
        lines.append('**Full run**: ' + reasons[ALL][0])
    else:
        lines.append('Selected TUs: **%d**' % len(selected))
        lines.append('')
        for tu in sorted(selected):
            lines.append('- `%s` <- %s' % (tu, ', '.join(sorted(set(reasons[tu])))))
    if elapsed is not None:
        lines.append('')
        lines.append('Selection took %.2f s' % elapsed)
    return '\n'.join(lines) + '\n'


def git(*args):
    """Stdout of a git command; raises on failure."""
    return subprocess.run(['git', *args], check=True, capture_output=True, text=True).stdout


def git_show(ref, path):
    """Return a file at a git ref, or '' when it doesn't exist there."""
    try:
        return git('show', ref + ':' + path)
    except subprocess.CalledProcessError:
        return ''


def main(argv=None):
    """Print the selection for base..head; 0 on success, 2 without TIDY_STRICT_FILES."""
    start = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', default='HEAD^1')
    parser.add_argument('--head', default='HEAD')
    parser.add_argument('--build-dir', default='build')
    parser.add_argument('--root', default='/ws', help='the repo path inside the depfiles')
    parser.add_argument('--format', choices=('list', 'regex'), default='list')
    parser.add_argument('--summary', help='append the Markdown report to this file')
    args = parser.parse_args(argv)

    changed = [p for p in git('diff', '--name-only', '--no-renames', args.base, args.head)
               .splitlines() if p]
    head_strict = strict_regex(git_show(args.head, 'Makefile'))
    if head_strict is None:
        print('No %s in the Makefile' % STRICT_VARIABLE, file=sys.stderr)
        return 2
    strict_changed = head_strict != strict_regex(git_show(args.base, 'Makefile'))

    texts = []
    for depfile in find_depfiles(args.build_dir):
        with open(depfile, encoding='utf-8') as f:
            texts.append(f.read())
    index = build_index(texts)
    with open(os.path.join(args.build_dir, 'compile_commands.json'), encoding='utf-8') as f:
        compiled = {entry['file'] for entry in json.load(f)}

    selected, reasons = select(changed, index, compiled, re.compile(head_strict), args.root,
                               strict_changed)
    text = report(changed, selected, reasons, time.monotonic() - start)
    sys.stderr.write(text)
    if args.summary:
        with open(args.summary, 'a', encoding='utf-8') as f:
            f.write(text)
    if selected == ALL:
        print(ALL)
    elif selected and args.format == 'regex':
        print(to_regex(selected))
    else:
        for tu in sorted(selected):
            print(tu)
    return 0


if __name__ == '__main__':
    sys.exit(main())
