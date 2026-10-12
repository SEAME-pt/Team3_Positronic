# Contributing to Positronic

How we work: with Claude, with Git and GitHub, and in code. Every rule says *why*, so any of us can defend it.

This file is the rule. To change a rule, open a PR that changes this file, and the config files with it if needed.

**Contents**
1. [Claude](#1-claude)
2. [Development workflow](#2-development-workflow)
3. [Coding standards](#3-coding-standards)

---

## 1. Claude

### 1.1 What we set up

The team has a shared **Project** in our Claude Team organization. Every chat inside it starts with the same background, so nobody has to re-explain the arm, SEA:ME or our rules.

| Piece | What it does | Source of truth |
|---|---|---|
| Name | `Positronic` | — |
| "What are you trying to achieve?" | One paragraph of the project's goal | §1.3 below |
| **Instructions** | *How* Claude behaves: teach first, explain the why, reply in your language, safety rules, offer ADRs | [`docs/claude/project-instructions.md`](claude/project-instructions.md) |
| **Knowledge files** | *What* Claude knows: program, hardware, team, decisions, status, our rules | [`docs/claude/positronic-context.md`](claude/positronic-context.md) and this file |
| GitHub repo | Claude reads our real code and docs | the repo, connected to the Project |

The files live **in the repo**, so changes to them go through a PR like any other change. *Why: one version everyone reviewed, instead of six slightly different copies.*

### 1.2 Using the shared Project

- **One topic per chat**, named after its issue, e.g. `#21 serial protocol`. *Why: a chat that mixes topics makes Claude lose track, and the issue number makes the chat easy to find again.*
- **Only the Scrum Master edits the Project** (instructions and knowledge files), and only by re-uploading files that are merged in the repo. Everyone else just uses it. *Why: if anyone edits it directly, the Project and the repo drift apart.*
- **At every sprint review**, the Scrum Master updates the "Current status" section of `positronic-context.md` in a PR, then re-uploads it.
- **Decisions don't stay in chats.** If a chat leads to a decision, ask Claude for an ADR (§2.10) and open a PR. A decision that's only in someone's chat history doesn't exist for the team.
- **Never upload** personal files, secrets, keys or datasets.

### 1.3 Recreating the Project yourself

You only need this for a personal copy. In Claude, go to **Projects → Create project**, then:

1. **Name:** `Positronic`
2. **What are you trying to achieve?**
   > Team Positronic (6 students, SEA:ME 2.0 at 42 Porto) is building the ROS 2 software for a myCobot 280 Jetson Nano arm. In M1 that means a ros2_control driver, MoveIt 2 motion in Gazebo and on the real arm, and an e-stop that always wins over our code. M2 adds vision, visual servoing, pick-and-place and learning from demonstration, and M3 adds multi-robot cooperation. We want to be the best team in the cohort and learn as much as possible, since we're new to robotics and embedded systems and are graded individually. We use Claude to learn concepts and why best practices exist, plan sprints and issues, review code, and keep every decision documented so we can defend it.
3. **Instructions:** paste the content of [`docs/claude/project-instructions.md`](claude/project-instructions.md).
4. **Knowledge:** upload [`docs/claude/positronic-context.md`](claude/positronic-context.md) and this `CONTRIBUTING.md`. Connect GitHub and add this repo.

### 1.4 Claude Code (terminal / IDE)

The repo has **no `CLAUDE.md`**. The whole setup lives in `docs/claude/` and in this file, so there's only one place to maintain. When you use Claude Code inside the repo, start the session with:

> *"Read `docs/claude/project-instructions.md`, `docs/claude/positronic-context.md` and `docs/CONTRIBUTING.md` and follow them."*

### 1.5 Getting the most out of it

| Do | Why |
|---|---|
| **Give context:** the issue number, the file, what you tried, and **where** it runs (laptop container or Jetson container) | Most wrong answers come from Claude guessing your setup |
| **Paste the full error and the command** that produced it | The cause is usually in the first error line, not the last |
| Say **how you want to learn**: "guide me, don't write it", "explain line by line", "give me hints" | By default it teaches first and only writes full code when you ask |
| Ask **"why?"** and **"what are the alternatives?"** | You'll defend each choice at the review |
| Ask **"what can go wrong on the real arm?"** before any motion test | Robots fail in ways you won't see in a terminal |
| Ask it to **review your diff** before opening a PR (paste `git diff origin/dev...`) | It catches convention slips before your reviewers do |
| Ask for the **artifacts**: ADR, PR description, test plan, acceptance criteria with numbers | That's where most of the documentation time goes |
| **Verify everything**: run it, measure it, check the **ROS 2 Jazzy** docs | Claude can be wrong, especially about ROS APIs that changed between distros. You own what you merge |
| Start a **new chat** when one gets long or changes topic | Answers degrade in long, mixed chats |

Example prompts:
- *"#22: I need a ros2_control hardware interface with a fake backend. Don't write it. Explain which methods I must implement and why, then let me try."*
- *"Here's my `make build` output (pasted). I'm in the laptop container. What's the first real error, and what does it mean?"*
- *"We chose 50 Hz for /joint_states after measuring the serial link (#20). Write an ADR for it, with the alternatives we considered."*

**Limits:** Claude can't see the arm, your screen or the Jetson. In the Project chat it can't run ROS or touch the hardware.

---

## 2. Development workflow

### 2.1 The flow at a glance

```
issue ──► branch from the issue ──► commits ──► PR to dev ──► 2 approvals + CI ──► squash merge
(template)   (base: dev)            (small,     (template,    (partner + one         (PR title =
                                    often)       draft early)  from another pair)      the commit)

end of sprint:  dev ──► PR to main ──► merge commit ──► tag v0.N.0 + GitHub Release
```

### 2.2 Branches

| Branch | What it holds | Who writes to it |
|---|---|---|
| `main` | States shown at a sprint review. Always works. | Only the end-of-sprint PR from `dev` |
| `dev` | Integration: everything finished and reviewed | Only squash-merged PRs |
| `<issue#>-<slug>` | One issue's work, e.g. `21-serial-protocol-library-in-c` | The issue's owner and their pair partner |

*Why two protected branches: `dev` can be briefly broken by an integration problem, while `main` is what we demo and tag, so it never is.*

### 2.3 Step by step

**1. Issue.** Every piece of work starts as an issue, created from a template:
- **User story/task:** one owner, one sprint, one PR.
- **Epic:** a goal that spans sprints.
- **Bug.**

Fill in the acceptance criteria **with numbers** and the **Safety** section. Set the **assignees** (the pair, with the owner first), the **milestone** `SPRINT NN`, the **labels** and the **epic link**.

*Why: the issue is the contract, and the reviewer checks the PR against its acceptance criteria.*

**2. Branch.** On the issue page, in the sidebar, click **Development → Create a branch**. Keep the suggested name and check that the source branch is `dev`. Then:
```bash
git fetch origin
git switch 21-serial-protocol-library-in-c
```
*Why: GitHub links the branch, the issue and the PR automatically, so the issue shows its progress.*

**3. Work.** Commit small and often (§2.4). Before every push:
```bash
make format && make format-check && make build && make test     # plus make tidy for C++
```
*Why: CI runs the same checks. Catching problems locally takes seconds; in CI it costs a round trip.*

**4. Stay in sync with `dev`.** Merge it in; never rebase a pushed branch:
```bash
git fetch origin
git merge origin/dev        # fix conflicts, then make build && make test, commit, push
```
*Why: rebasing rewrites history and needs `git push --force`, which can destroy your pair partner's commits. The extra merge commits disappear when we squash (step 7).*

**5. Pull request into `dev`.**
- Open it **early, as a Draft**, so your pair and the reviewers can follow along. Mark it "Ready for review" when it's done.
- The title is a Conventional Commit (§2.4).
- Fill in every template section (§2.5) and write `Closes #<issue>`.
- Request **two reviewers: your pair partner and one person from another pair.**

**6. Review.** You need **2 approvals and green CI** (§2.6). Push fixes as new commits; never force-push during a review. *Why: reviewers see exactly what changed since they last looked.*

**7. Squash merge.** The **author** merges after both approvals (§2.7) and deletes the branch. The issue closes by itself thanks to `Closes #n`.

### 2.4 Commit messages and PR titles: Conventional Commits

```
type(scope): short imperative summary

Optional body: why this change, not what (the diff shows what).
```

| type | When | Example |
|---|---|---|
| `feat` | New behaviour | `feat(hardware): read joint angles from the serial port` |
| `fix` | Bug fix | `fix(hardware): convert degrees to radians before publishing` |
| `test` | Tests only | `test(protocol): cover checksum of the get_angles frame` |
| `docs` | Docs only | `docs(adr): choose 50 Hz for joint states` |
| `refactor` | Same behaviour, better code | `refactor(protocol): split frame parsing from I/O` |
| `perf` | Faster or lighter | `perf(hardware): reuse the read buffer` |
| `build` | Docker, CMake, dependencies | `build(docker): add ros-gz to the dev image` |
| `ci` | GitHub Actions | `ci: run colcon test in the team image` |
| `chore` | Anything else (settings, templates) | `chore: add ADR template` |

- **Scope:** the package without the `positronic_` prefix (`hardware`, `description`, `moveit_config`, `bringup`) or the area (`docker`, `ci`, `jetson`, `adr`, `standups`). Optional.
- **Summary:** imperative ("add", not "added"), lowercase, no full stop, 72 characters at most.
- **Breaking change:** add `!`, as in `feat(hardware)!: publish positions in radians`.

**What's enforced is the PR title**, because squash merge turns it into the commit that lands on `dev`. Write branch commits in the same style anyway: your pair reads them.

*Why: the history reads like a changelog, `git log --oneline` tells the story of a sprint, and the types group the release notes.*

### 2.5 Pull requests

| Template section | What to write |
|---|---|
| What changes | 2–5 bullets |
| Why | The problem, plus `Closes #n` |
| How to test | Exact commands a reviewer can copy, e.g. `make build && make test`, then `ros2 launch …` |
| Evidence | Numbers, plots, screenshots or logs, e.g. `ros2 topic hz /joint_states: 50.1 Hz, max period 23 ms` |
| Safety impact | `none` / `motion` / `e-stop` / `limits` / `hardware`. If it's not `none`, say what you checked and the safe state |
| Checklist | Honest ticks. If you can't tick one, explain why |

PRs stay **under 400 changed lines**. Generated files and meshes don't count. *Why: review quality collapses on big PRs, so split the work into steps.*

### 2.6 Reviews

- **Two approvals:** your **pair partner** and **one person from another pair**. *Why: the partner checks that it's correct; the outsider checks that it's understandable and spreads the knowledge. If only your pair understands the code, nobody else can defend it.*
- **How to review:**
  1. Read the issue's acceptance criteria.
  2. Check out the branch and run "How to test".
  3. Read the diff.
  4. Comment, then **Approve** or **Request changes**.
- **Prefix your comments:**
  - `issue:` must be fixed (blocking).
  - `question:` explain it to me.
  - `suggestion:` consider this (not blocking).
  - `nit:` tiny or style (not blocking).
- **Check:** correctness, safety (limits, units, timeouts), tests, naming and style, docs, and that the PR does only what its issue asks.
- The person who **opened a thread resolves it** once they're satisfied.
- Review within **one working day**. *Why: a PR waiting two days blocks a whole pair.*

### 2.7 Squash merging: how

On the PR page, once there are 2 approvals and CI is green:

1. Click the arrow next to the merge button and pick **Squash and merge**.
2. **Commit title:** check that it's the PR title in Conventional Commits form. GitHub appends `(#PR)`.
3. **Commit message:** delete the list of branch commits ("wip", "fix typo"…). Leave it empty, or write one or two lines of *why*.
4. Click **Confirm squash and merge**, then **Delete branch**.

*Why squash: one issue becomes one commit on `dev`. It's easy to read, and `git revert` undoes a whole feature in one step.*

### 2.8 End of sprint: `dev` → `main`

After the sprint review, the Scrum Master:

1. Opens a PR from `dev` to `main` titled `chore(release): sprint NN`, with the usual 2 approvals.
2. Merges it with **Create a merge commit**, not squash. *Why: `main` keeps the individual feature commits.*
3. Creates the tag and release: **Releases → Draft a new release → tag `v0.N.0`** (N = sprint number) **→ Generate release notes → Publish**.

*Why: every demo points to an exact version that anyone can check out and rerun.*

### 2.9 Standups

- After each daily, the Scrum Master writes `docs/standups/YYYY-MM-DD.md` from [`TEMPLATE.md`](standups/TEMPLATE.md) and commits it to the branch `standups-sprint-NN`, created from the sprint's standups issue.
- On the **last day of each week**, that branch goes to `dev` in **one PR** titled `docs(standups): sprint NN week W`, with the normal 2 approvals and squash merge.

*Why: `dev` keeps its protection with no exceptions, and reviewing a week of standups at once costs a few minutes instead of a PR every day.*

### 2.10 Decisions: ADRs

Every technical decision gets an **ADR** (Architecture Decision Record) in `docs/decisions/NNNN-short-title.md`, merged through a PR:

```markdown
# NNNN. Title
Date, status (proposed / accepted / superseded by NNNN), deciders.
## Context        what problem, what constraints, what we measured
## Decision       what we chose, in one or two sentences
## Alternatives   what else we considered and why not
## Consequences   what gets easier, what gets harder
## Revisit when   the signal that would make us change it
```

*Why: at the review, "why did you do it like this?" is answered by an ADR written when the decision was made, not reconstructed from memory.* If a decision changes, write a new ADR that supersedes the old one. Never edit an accepted ADR.

### 2.11 Repository settings (admin)

- **Settings → General → Pull Requests:**
  - Allow **squash merging**, with default message "Pull request title".
  - Allow **merge commits** (only for `dev` → `main`).
  - **Disable** rebase merging.
  - Enable **Automatically delete head branches**.
- **Ruleset for `dev`:**
  - Require a PR with **2 approvals**.
  - Dismiss stale approvals when new commits are pushed.
  - Require the **CI** status check.
  - Block force pushes and deletion.
  - Allowed merge method: **squash**.
- **Ruleset for `main`:** the same, but the allowed merge method is **merge**.
- **Actions:** require approval for workflows from outside contributors, and never run hardware jobs on `pull_request` events from forks. *Why: in a public repo, a stranger's fork could otherwise run code on our arm.*

---

## 3. Coding standards

Our style is the **ROS 2 code style, as enforced by the ROS 2 Jazzy lint tools** (`ament_lint_common`), plus our [`.clang-format`](../.clang-format) and [`.clang-tidy`](../.clang-tidy). Tools enforce almost everything, so you don't have to memorise it.

### 3.1 Commands

| Command | What it does |
|---|---|
| `make format` | Rewrites C/C++ in `src/`: **clang-format** with our `.clang-format`, then **uncrustify** with ROS's config |
| `make format-check` | Runs the ROS lint tools on `src/` without changing anything: uncrustify, cpplint, cppcheck, flake8, pep257, copyright ([`scripts/lint.sh`](../scripts/lint.sh)) |
| `make build` | `colcon build` |
| `make test` | `colcon test`: unit tests, plus each package's linters (the same tools, plus `lint_cmake` and `xmllint`) |
| `make tidy` | clang-tidy with our `.clang-tidy`. Run `make build` first |

**Why two formatters.** In `make test`, ROS checks C++ formatting with **uncrustify**, not clang-format. Our `.clang-format` is ROS's own clang-format config (`ament_clang_format`) plus a few options that make it agree with uncrustify. In a few places clang-format physically can't produce what uncrustify demands: the body of a lambda, the continuation lines of a wrapped `while`/`for` condition, and `case` blocks with braces. So `make format` runs clang-format first and uncrustify last. Uncrustify has the final word, and the result passes `make test`.

**In Zed** (or any editor using clangd), format-on-save applies `.clang-format`. That's nearly always correct. Run `make format` before committing anyway, so uncrustify can fix the exceptions above.

### 3.2 C++ formatting

This is real output of `make format` that passes every check:

```cpp
// Copyright 2026 Positronic
//
// Permission is hereby granted, free of charge, ... (full MIT header, see §3.3)

#ifndef POSITRONIC_HARDWARE__SERIAL_PORT_HPP_
#define POSITRONIC_HARDWARE__SERIAL_PORT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace positronic_hardware
{

enum class JointId : std::uint8_t
{
  JOINT_1 = 1,
  JOINT_2,
  JOINT_3
};

/// Owns the serial file descriptor and closes it in the destructor (RAII).
class SerialPort
{
public:
  explicit SerialPort(const std::string & device);
  ~SerialPort();
  bool write_frame(const std::vector<std::uint8_t> & frame);

private:
  int fd_{-1};
  std::string device_;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__SERIAL_PORT_HPP_
```

```cpp
#include "positronic_hardware/serial_port.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace positronic_hardware
{

namespace
{
constexpr std::size_t MAX_FRAME_SIZE = 64;
}  // namespace

SerialPort::SerialPort(const std::string & device)
: device_(device)
{
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY);
}

SerialPort::~SerialPort()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool SerialPort::write_frame(const std::vector<std::uint8_t> & frame)
{
  const auto is_invalid = [](std::uint8_t byte) {
      return byte == 0xFF;
    };
  if (
    fd_ < 0 || frame.empty() || frame.size() > MAX_FRAME_SIZE ||
    std::any_of(frame.begin(), frame.end(), is_invalid))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("serial_port"), "Rejected frame of %zu bytes on %s", frame.size(),
      device_.c_str());
    return false;
  }
  const auto written = ::write(fd_, frame.data(), frame.size());
  return written == static_cast<ssize_t>(frame.size());
}

}  // namespace positronic_hardware
```

| Rule | Example | Why |
|---|---|---|
| **2 spaces**, no tabs | | ROS 2 / Google style |
| **100 columns** max | | ROS 2 limit (cpplint and uncrustify both check it) |
| Braces on **their own line** after `namespace`, `class`, `struct`, `enum` and function definitions | `class SerialPort` / `{` | ROS 2 style |
| Braces on the **same line** for `if`, `else`, `for`, `while`, `switch`, `do` | `if (x) {` | ROS 2 style |
| …except when the condition wraps onto several lines: then `if (` breaks and `{` goes on its own line | see `write_frame` above | Keeps the condition visually separate from the body |
| **Braces are always required**, even for one-line bodies | `if (fd_ >= 0) {` | ROS 2 rule. The Apple "goto fail" bug was a line someone thought was inside an unbraced `if` |
| Pointer and reference symbols have a space on **both sides** | `int * p`, `const std::string & s` | ROS 2 style |
| Constructor initializer list on its own line, starting with `:` | `: device_(device)` | ROS 2 style |
| No space after `template` | `template<typename T>` | ROS 2 style |
| Lambda bodies always go on their own lines, indented as in the example | `[](auto b) {` | Required by uncrustify |
| One-line functions only when the body is empty | `void reset() {}` | Easier to read and to debug with breakpoints |
| Wrapped call arguments break after `(` and indent by 2 | `RCLCPP_WARN(` | ROS 2 style |
| `// namespace x` comment after a namespace closes | `}  // namespace positronic_hardware` | Required by cpplint |
| `TODO` comments carry a name | `// TODO(feazeved): handle timeout` | Required by cpplint, so every TODO has an owner |

### 3.3 Files, headers and includes

- **Copyright header on every source file** (`.cpp`, `.hpp`, `.py`, `.cmake`): the full MIT text that `ament_copyright` checks. Don't type it; add it with:
  ```bash
  ament_copyright --add-missing "Positronic" mit src/<package>     # inside make shell
  ```
  *Why: ROS's copyright linter fails any file without a recognised licence header.*
- **Header guards, not `#pragma once`**, named after the path inside `include/`, with `__` for each `/`:
  `include/positronic_hardware/serial_port.hpp` → `POSITRONIC_HARDWARE__SERIAL_PORT_HPP_`, and the `#endif` comment repeats the name. *Why: cpplint requires guards in exactly this format.*
- **File names:** `snake_case.hpp` / `.cpp`. Public headers go in `include/<package>/` and sources in `src/`.
- **Include order**, with a blank line between groups (cpplint checks this):
  1. The file's own header: `"positronic_hardware/serial_port.hpp"`.
  2. C system headers: `<fcntl.h>`.
  3. C++ standard library: `<vector>`.
  4. Other libraries, in quotes as ROS does: `"rclcpp/rclcpp.hpp"`.
  5. Our other packages: `"positronic_protocol/frame.hpp"`.

  clang-format sorts the includes inside each group.
- **No `using namespace`** (cpplint). The only exception is literal namespaces in `.cpp` files, e.g. `using namespace std::chrono_literals;` for `500ms`. Use declarations instead: `using std::string;`. *Why: a using-directive pulls a whole namespace into scope and causes silent name clashes.*

### 3.4 Naming (checked by `make tidy`)

| Thing | Style | Example |
|---|---|---|
| Classes, structs, enums, template parameters | `CamelCase` | `SerialPort`, `JointId`, `typename Backend` |
| Functions, methods, variables, parameters | `snake_case` | `read_angles()`, `joint_count` |
| Private and protected members | `snake_case_` | `fd_`, `device_` |
| Namespaces | `lower_case` | `positronic_hardware` |
| `constexpr`, global constants, enum values, macros | `UPPER_CASE` | `MAX_FRAME_SIZE`, `JointId::JOINT_1` |
| Packages | `positronic_<role>` | `positronic_hardware`, `positronic_bringup` |

*Why the trailing `_`: inside a method you can tell a member from a local at a glance, without `this->`.*

Never name a constant or enum value after a common macro (`ERROR`, `OK`, `TRUE`, `DEBUG`). *Why: `UPPER_CASE` names collide with macros from system headers, and the compile error is confusing.*

### 3.5 Static analysis (`make tidy`, cppcheck)

**Enabled clang-tidy families:**

| Family | Catches |
|---|---|
| `bugprone-*` | Common bugs: wrong `sizeof`, suspicious semicolons, use after move |
| `cert-*` | Secure-coding rules (CERT) |
| `clang-analyzer-*` | Deep path analysis: null dereference, leaks, uninitialized reads |
| `concurrency-*` | Thread-unsafe functions |
| `cppcoreguidelines-*` | C++ Core Guidelines: ownership, casts, initialization |
| `misc-*` | Assorted, including "this local could be `const`" |
| `modernize-*` | C++11/14/17 features (`nullptr`, `auto`, `override`, `using`) |
| `performance-*` | Needless copies, inefficient algorithms |
| `portability-*` | Code that only works on one platform |
| `readability-*` | Naming, braces, implicit conversions, function size |

**Turned off on purpose:**
- Magic numbers: joint limits and gains are numbers by nature.
- Short identifier names: `q`, `dt` and `i` are normal in robotics.
- Checks that fire on every ROS log macro (varargs, `do/while`).
- Trailing return types.
- `[[nodiscard]]` everywhere.
- Include-cleaner.
- Easily swappable parameters.

**Limits:** a function is flagged above **80 lines** or **6 parameters**.

**The rule:**
- A PR adds **no new clang-tidy or cppcheck warnings** in the files it touches.
- If a warning is truly wrong, silence that one line with `// NOLINT(check-name)` and explain why in the PR.
- Reviewers check this.

*Why: tidy is configured to warn, not fail, so the discipline comes from the review. A warning that's ignored once gets ignored forever.*

**42 habits the tools will flag:**

| 42 habit | Write instead | Why |
|---|---|---|
| `int buf[64];` | `std::array<std::uint8_t, 64> buf{};` | Knows its size; `.at()` is bounds-checked |
| `malloc`/`free`, `new`/`delete` | Containers, `std::make_unique` | Freed automatically, even on early returns (RAII) |
| `(int)x` | `static_cast<int>(x)` | Visible, searchable, and checked by the compiler |
| `NULL` | `nullptr` | Typed. `NULL` is just `0` |
| `int x;` | `int x{0};` | Reading uninitialized memory is undefined behaviour |
| `printf` | `RCLCPP_INFO(...)` | Goes to ROS logs, with timestamps and levels |
| `if (count)` on an integer | `if (count != 0)` | States the intent (`if (ptr)` on pointers is fine) |
| `int i = some_double;` | `static_cast<int>(std::lround(d))` | Silent truncation hides unit and rounding bugs |
| `long`, `short` | `std::int64_t`, `std::int16_t` | Exact sizes. cpplint flags `long`/`short` (`runtime/int`) |

### 3.6 C++ practices

- **C++17** (`set(CMAKE_CXX_STANDARD 17)`). *Why: it's ROS 2 Jazzy's target standard, so every ROS header and example is tested with it.*
- **Real-time path:** inside the ros2_control `read()`/`write()` loop:
  - no memory allocation (no `new`, no growing containers, no building `std::string`s);
  - no exceptions;
  - no blocking beyond the serial I/O itself;
  - report failures with `return_type::ERROR`.

  *Why: this loop runs at a fixed rate, and allocations or exceptions stall it unpredictably. That's exactly the jitter M1 asks us to measure.* Exceptions are fine outside this path (configuration, startup).
- **Ownership:**
  - No raw owning pointers.
  - `std::unique_ptr` by default; `std::shared_ptr` only when ownership is really shared.
  - Wrap OS resources in RAII classes (`SerialPort` closes its file descriptor in the destructor).
- **Units:** SI on every ROS interface: metres, radians, seconds (REP 103). The arm's degrees and millimetres exist **only inside the driver**. *Why: on a robot, a unit bug is a crash, not a wrong number on a screen.*
- **Comments** explain *why*, not what. Public headers in `include/` document each class and public method with `///` (Doxygen).
- **Logging:**
  - Use `RCLCPP_*` macros only.
  - Inside loops, use the `_THROTTLE` variants, e.g. `RCLCPP_WARN_THROTTLE(logger, clock, 1000, ...)`.

  *Why: logging every cycle of a 50 Hz loop floods the console and costs time.*
- Every node that can move the arm is a **lifecycle node**.

### 3.7 Creating a package

Use `ros2 pkg create` inside `make shell`. It generates the lint setup for you:

```bash
ros2 pkg create --build-type ament_cmake --license MIT positronic_<role>     # C++
ros2 pkg create --build-type ament_python --license MIT positronic_<role>    # Python
```

C++ packages keep this in `CMakeLists.txt`. It runs the **full** ROS lint set in `make test`:

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

`package.xml` contains `<license>MIT</license>`, plus `<test_depend>ament_lint_auto</test_depend>` and `<test_depend>ament_lint_common</test_depend>`.

Don't exclude any linter. *Why: ROS 2 lint is our standard, and `make format-check` and `make test` must agree.*

### 3.8 Python

Python is checked by ROS's **`ament_flake8`** and **`ament_pep257`**, both in `make format-check` and in `make test`.

| Rule | Example | Why |
|---|---|---|
| **Python 3.12** | | The version in our Jazzy image: code runs where it's tested |
| **99 columns** max | | ROS's flake8 config |
| **Single quotes** for strings | `'joint_reader'` | `flake8-quotes`, as configured by ROS |
| Imports grouped **standard library → third party → ours**, alphabetical, with a blank line between groups | `import os` / blank / `import rclpy` | `flake8-import-order` (Google style) |
| Don't shadow builtins | not `def read(list):` | `flake8-builtins` |
| Docstrings: when you write one, it starts with a capital letter and ends with a period | `"""Read the joint angles."""` | `pep257`, ROS convention. Missing docstrings are allowed |
| No unused imports or variables | | `pyflakes` |
| Naming: `snake_case` functions and variables, `CamelCase` classes, `UPPER_CASE` constants | | PEP 8 |
| Tests with **pytest**, in each package's `test/` folder | | Run by `make test` |

There's no Python auto-formatter: `make format-check` lists each problem with its file and line, and you fix it by hand. *Why: black, the usual formatter, uses double quotes, which ROS's flake8 rejects.*
