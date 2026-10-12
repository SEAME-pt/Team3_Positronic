# Contributing to Positronic

How we work: with Claude, with Git and GitHub, and in code. Every rule says why, so we can defend it.

> **Status: draft for the team discussion on 12 Oct 2026.** Items marked **[DISCUSS]** are open.
> Tick an option, then a PR updates this file and the config files to match. After that, this file is
> the rule. To change a rule later, open a PR that changes it.

**Contents**
1. [Claude](#1-claude)
2. [Development workflow](#2-development-workflow)
3. [Coding standards](#3-coding-standards)

---

## 1. Claude

### 1.1 What we set up

The team has a shared **Project** in our Claude Team organization. A Project gives every chat inside it the same background, so nobody has to re-explain the arm, SEA:ME or our rules.

| Piece | What it does | Source of truth |
|---|---|---|
| Name | `Positronic` | — |
| "What are you trying to achieve?" | One paragraph of the project's goal, written when the Project was created | §1.3 below |
| **Instructions** | *How* Claude behaves: teach first, explain the why, reply in your language, safety rules, offer ADRs | [`docs/claude/project-instructions.md`](claude/project-instructions.md) |
| **Knowledge files** | *What* Claude knows: program, hardware, team, decisions, status | [`docs/claude/positronic-context.md`](claude/positronic-context.md) and this file |
| GitHub repo (once it's public) | Claude can read our real code and docs | the repo |

The files live **in the repo**, so changes to them go through a PR like any other change. *Why: one version everyone reviewed, instead of six slightly different copies.*

### 1.2 Using the shared Project

- **One topic per chat.** Name it after the issue, e.g. `#21 serial protocol`. *Why: long chats that mix topics make Claude lose track, and you can find a chat again by its issue number.*
- **Only the Scrum Master edits the Project** (instructions and knowledge), and only by re-uploading files merged in the repo. Everyone else just uses it. *Why: if anyone edits it directly, the Project and the repo drift apart.*
- **Update it at every sprint review.** The Scrum Master updates the "Current status" section of `positronic-context.md` in a PR, then re-uploads it.
- **Decisions don't stay in chats.** If a chat leads to a decision, ask Claude for an ADR (§2.10) and open a PR. A decision that's only in someone's chat history doesn't exist for the team.
- **Don't upload** personal files, secrets, keys or big datasets to the Project.

### 1.3 Recreating it yourself

You only need this for your own copy, for example to experiment with different instructions. In Claude: **Projects → Create project**, then:

1. **Name:** `Positronic`
2. **What are you trying to achieve?**
   > Team Positronic (6 students, SEA:ME 2.0 at 42 Porto) is building the ROS 2 software for a myCobot 280 Jetson Nano arm. In M1 that means a ros2_control driver, MoveIt 2 motion in Gazebo and on the real arm, and an e-stop that always wins over our code. M2 adds vision, visual servoing, pick-and-place and learning from demonstration, and M3 adds multi-robot cooperation. We want to be the best team in the cohort and learn as much as possible, since we're new to robotics and embedded systems and are graded individually. We use Claude to learn concepts and why best practices exist, plan sprints and issues, review code, and keep every decision documented so we can defend it.
3. **Instructions:** paste the content of [`docs/claude/project-instructions.md`](claude/project-instructions.md).
4. **Knowledge:** upload [`docs/claude/positronic-context.md`](claude/positronic-context.md) and this `CONTRIBUTING.md`.
5. Once the repo is public, connect GitHub in the Project's knowledge and add the repo (at least `docs/`).

### 1.4 Claude Code and `CLAUDE.md`

[`CLAUDE.md`](../CLAUDE.md) at the repo root is read automatically by **Claude Code**, Claude running in your terminal or IDE inside the repo. It's deliberately short: commands, safety rules, and "follow `docs/CONTRIBUTING.md`". *Why: Claude Code then follows our conventions without anyone pasting them, and since it points here, there's still one source of truth.*

### 1.5 Getting the most out of it

| Do | Why |
|---|---|
| **Give context:** the issue number, the file, what you tried, and **where** it runs (your laptop container, or the Jetson container) | Most wrong answers come from Claude guessing your setup |
| **Paste the full error and the command** that produced it, not "it doesn't work" | The cause is usually in the first error line, not the last |
| Say **how you want to learn**: "guide me, don't write it", "explain line by line", "give me hints" | By default it teaches first and only writes full code when you ask |
| Ask **"why?"** and **"what are the alternatives?"** | You'll have to defend each choice at the review |
| Ask **"what can go wrong on the real arm?"** before any motion test | Robots fail in ways you won't see in a terminal |
| Ask it to **review your diff** before you open a PR (paste `git diff dev...`) | It catches convention slips before your reviewers do |
| Ask for the **artifacts**: ADR, PR description, test plan, acceptance criteria with numbers | That's where most of the documentation time goes |
| **Verify everything.** Run it, measure it, read the docs for **ROS 2 Jazzy** | Claude can be wrong, especially about ROS APIs that changed between distros. You own what you merge |
| Start a **new chat** when one gets long or changes topic | Answers degrade in long, mixed chats |

Example prompts:
- *"#22: I need a ros2_control hardware interface with a fake backend. Don't write it. Explain which methods I must implement and why, then let me try."*
- *"Here's my `make build` output (pasted). I'm in the laptop container. What's the first real error, and what does it mean?"*
- *"We chose 50 Hz for /joint_states after measuring the serial link (#20). Write an ADR for it with the alternatives we considered."*

**Limits:** Claude can't see the arm, your screen or the Jetson. In the Project chat it can't run ROS or touch the hardware.

---

## 2. Development workflow

### 2.1 The flow at a glance

```
issue ──► branch from the issue ──► commits ──► PR to dev ──► 2 reviews + CI ──► squash merge
 (template)   (base: dev)            (small,     (template,     (partner + one       (title =
                                     often)       draft early)   from another pair)    commit msg)

end of sprint:  dev ──► PR to main ──► merge commit ──► tag v0.N.0 + GitHub Release
```

### 2.2 Branches

| Branch | What it holds | Who writes to it |
|---|---|---|
| `main` | States shown at a sprint review. Always works. | Only the end-of-sprint PR from `dev` |
| `dev` | Integration: everything finished and reviewed | Only squash-merged PRs |
| `<issue#>-<slug>` | One issue's work, e.g. `21-serial-protocol-library-in-c` | The issue's owner (and their pair) |

*Why two protected branches: `dev` can be briefly broken by an integration problem, while `main` is what we demo and tag, so it never is.*

### 2.3 Step by step

**1. Issue.** Every piece of work starts as an issue, created from a template:
- **User story/task:** one person, one sprint, one PR.
- **Epic:** a goal that spans sprints.
- **Bug.**

Fill in the acceptance criteria **with numbers** and the **Safety** section. Set:
- **Assignee:** the owner. In M1, one owner per story inside the pair.
- **Milestone:** `Sprint NN`.
- **Labels.**
- **Epic link.**

*Why: the issue is the contract. The reviewer checks the PR against its acceptance criteria.*

**2. Branch.** On the issue page, in the sidebar, click **Development → Create a branch**. Keep the suggested name and check that the source branch is `dev`. Then:
```bash
git fetch origin
git switch 21-serial-protocol-library-in-c
```
*Why: GitHub links the branch, the issue and the PR automatically, so the issue shows its progress.*

**3. Work.** Commit small and often (§2.4). Before every push:
```bash
make format && make build && make test     # and make tidy for C++ changes
```
*Why: CI runs the same checks. Catching problems locally takes seconds; catching them in CI takes a round trip.*

**4. Stay in sync with `dev`.** Merge it in; don't rebase:
```bash
git fetch origin
git merge origin/dev        # fix conflicts, make build && make test, commit, push
```
*Why merge and not rebase: rebase rewrites history and needs `git push --force`, which can destroy a teammate's commits on a shared pair branch. The extra merge commits disappear anyway when we squash (step 7).*

**5. Pull request into `dev`.**
- Open it **early, as a Draft**, so your pair and the reviewers can follow along. Mark it "Ready for review" when it's done.
- The title is a Conventional Commit (§2.4).
- Fill in every section of the template (§2.5) and write `Closes #<issue>`.
- Request **two reviewers: your pair partner and one person from another pair.**

**6. Review.** You need **2 approvals and green CI** (§2.6). Push fixes as new commits; don't force-push during a review. *Why: reviewers see exactly what changed since their last look.*

**7. Squash merge.** The **author** merges after the approvals (§2.7). Delete the branch. The issue closes by itself thanks to `Closes #n`.

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
| `chore` | Anything else (repo settings, templates) | `chore: add ADR template` |

- **Scope:** the package without the `positronic_` prefix (`hardware`, `description`, `moveit_config`, `bringup`) or the area (`docker`, `ci`, `jetson`, `adr`). It's optional.
- **Summary:** imperative ("add", not "added"), lowercase, no full stop, 72 characters at most.
- **Breaking change:** add `!`, as in `feat(hardware)!: publish positions in radians`.

**What's enforced:** the **PR title**, because squash merge turns it into the one commit that lands on `dev`. Commits inside your branch are yours, but use the same style anyway: it's a good habit, and your pair reads them.

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

Keep PRs **under 400 changed lines** (generated files and meshes don't count). *Why: review quality collapses on big PRs. Split the work into steps instead.*

### 2.6 Reviews

- **Two approvals:** your **pair partner** and **one person from another pair**. *Why: the partner checks that it's correct; the outsider checks that it's understandable and spreads the knowledge. If only your pair understands the code, nobody else can defend it.*
- **How to review:**
  1. Read the issue's acceptance criteria.
  2. Check out the branch and run "How to test".
  3. Read the diff.
  4. Comment, then **Approve** or **Request changes**.
- **Prefix your comments** so the author knows what's blocking:
  - `issue:` must be fixed (blocking).
  - `question:` explain it to me.
  - `suggestion:` consider this (not blocking).
  - `nit:` tiny or style (not blocking).
- **Check:** correctness, safety (limits, units, timeouts), tests, naming and style, docs, and whether the PR does more than its issue asks.
- The person who **opened a thread resolves it**, once they're satisfied.
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

1. Opens a PR from `dev` to `main` titled `chore(release): sprint NN`.
2. Merges it with **Create a merge commit**, not squash. *Why: `main` keeps the individual feature commits.*
3. Creates the tag and release: **Releases → Draft a new release → tag `v0.N.0`** (N = sprint number) **→ Generate release notes → Publish**.

*Why: every demo points to an exact version that anyone can check out and rerun.*

### 2.9 Standups **[DISCUSS]**

The template says to commit standups directly to `dev`, but `dev` is protected (PR required). Pick one:

- [ ] **One docs PR per day, fast-tracked.** The Scrum Master opens it and any one approval merges it. Branch protection stays as it is.
- [ ] **Direct pushes to `dev` for the Scrum Master** (a branch-protection bypass for one role). Simpler, but it's easy to slip code in by mistake.
- [ ] **One PR per week** with all of that week's standups, collected on a `standups-sprint-NN` branch.
- [ ] Other: ______________________

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

*Why: at the review, "why did you do it like this?" is answered by an ADR written when the decision was made, not reconstructed from memory. If a decision changes, write a new ADR that supersedes the old one, and never edit an accepted one.*

### 2.11 Repository settings (admin checklist)

- [ ] **Settings → General → Pull Requests:**
  - Allow **squash merging**, with default message "Pull request title".
  - Allow **merge commits** (only for `dev` → `main`).
  - Untick rebase merging.
  - Tick **Automatically delete head branches**.
- [ ] **Rulesets for `dev`:**
  - Require a PR with **2 approvals**.
  - Dismiss stale approvals when new commits are pushed.
  - Require the **CI** status check.
  - Block force pushes.
  - Allowed merge method: **squash**.
- [ ] **Rulesets for `main`:** the same, plus allowed merge method **merge**.
- [ ] **Actions (public repo, before any self-hosted runner touches the arm):** require approval for workflows from outside contributors, and never run hardware jobs on `pull_request` from forks.

---

## 3. Coding standards

Tools enforce most of these, so you don't have to remember them:

| Command | Does | Config |
|---|---|---|
| `make format` | Rewrites C++ files to our style | [`.clang-format`](../.clang-format) (clang-format **18.1.8**, pinned) |
| `make format-check` | Same, without changing anything (this is what CI runs) | same |
| `make tidy` | Static analysis and naming checks (run `make build` first) | [`.clang-tidy`](../.clang-tidy) |
| `make test` | Unit tests and the ROS linters of each package | each package's `CMakeLists.txt` |

*Why pin the formatter version: different clang-format versions format the same code slightly differently. Pinning means everyone, and CI, gets identical results.*

### 3.1 C++ formatting (`.clang-format`)

Our base is the **ROS 2 style**: Google style with ROS's changes. *Why: it's what the ROS 2 code we read and copy from looks like.* Here's what it produces:

```cpp
namespace positronic_hardware
{

class SerialPort
{
public:
  explicit SerialPort(const std::string & device);
  bool write_frame(const std::vector<std::uint8_t> & frame);

private:
  int fd_{-1};
};

bool SerialPort::write_frame(const std::vector<std::uint8_t> & frame)
{
  if (frame.empty()) {
    return false;
  }
  const auto written = ::write(fd_, frame.data(), frame.size());
  return written == static_cast<ssize_t>(frame.size());
}

}  // namespace positronic_hardware
```

**F1. Indentation: 2 spaces, no tabs** (Google/ROS). The 42 Norm used tabs.
- [ ] Keep 2 spaces
- [ ] 4 spaces

**F2. Line length: 100 columns** (ROS 2).
- [ ] Keep 100
- [ ] 80 (Google, easier side-by-side diffs)
- [ ] 120

**F3. Brace placement:** braces go on **their own line** after `class`, `struct`, `namespace` and function definitions, and on the **same line** for `if`/`for`/`while`/`switch` (ROS 2).
- [ ] Keep (ROS 2)
- [ ] Same line everywhere (Google)
- [ ] Own line everywhere (Allman)

**F4. Braces are always required.** `InsertBraces` adds them around one-line bodies, and one-line `if (x) return;` isn't allowed. *Why: the classic Apple "goto fail" bug was a second line someone thought was inside an unbraced `if`.*
- [ ] Keep (always braces)
- [ ] Allow single-line `if`/`for` without braces

**F5. Pointer and reference alignment: `int * p`, `const T & x`** (`Middle`, ROS 2).
- [ ] Keep `Middle` (`int * p`)
- [ ] `Left` (`int* p`, the most common in modern C++: "the type is `int*`")
- [ ] `Right` (`int *p`, the C and 42 Norm habit)

**F6. One-line functions:** only empty bodies (`void f() {}`). Everything else gets a full body.
- [ ] Keep
- [ ] Also allow short inline functions in class bodies (`AllowShortFunctionsOnASingleLine: Inline`)

**F7. Include order:** clang-format sorts includes *within* each blank-line-separated group. We propose these groups, in this order:
1. The file's own header.
2. C system headers (`<fcntl.h>`).
3. C++ standard library (`<vector>`).
4. Other libraries (`<rclcpp/rclcpp.hpp>`).
5. Our packages (`"positronic_hardware/…"`).

- [ ] Agree
- [ ] Other: ______________________

### 3.2 C++ naming (enforced by `make tidy`)

| Thing | Style | Example |
|---|---|---|
| Classes, structs, enums, template parameters | `CamelCase` | `SerialPort`, `JointState`, `typename Backend` |
| Functions, methods, variables, parameters | `snake_case` | `read_angles()`, `joint_count` |
| Private and protected members | `snake_case_` (trailing underscore) | `fd_`, `max_rad_` |
| Namespaces | `lower_case` | `positronic_hardware` |
| `constexpr`, global constants, enum values, macros | `UPPER_CASE` | `BAUD_RATE`, `JointId::JOINT_1` |
| Files | `snake_case.hpp` / `.cpp`, headers in `include/<package>/` | `serial_port.hpp` |

*Why the trailing `_`: inside a method you can tell a member from a local at a glance, without `this->`.*

**N1. Constants and enum values: `UPPER_CASE`.** The alternative is Google's `kCamelCase` (`kBaudRate`). Google uses it because `UPPER_CASE` can collide with macros: an enum value called `ERROR` breaks if some header defines `#define ERROR`.
- [ ] Keep `UPPER_CASE`
- [ ] `kCamelCase`

### 3.3 C++ static analysis (`.clang-tidy`)

**Enabled check families:**

| Family | Catches |
|---|---|
| `bugprone-*` | Common bugs: wrong `sizeof`, suspicious semicolons, use after move |
| `cert-*` | Secure-coding rules (CERT) |
| `clang-analyzer-*` | Deep path analysis: null dereference, leaks, uninitialized reads |
| `concurrency-*` | Thread-unsafe functions |
| `cppcoreguidelines-*` | The C++ Core Guidelines (Stroustrup and Sutter): ownership, casts, initialization |
| `misc-*` | Assorted, including "this local could be `const`" |
| `modernize-*` | Use C++11/14/17 features (`nullptr`, `auto`, `override`, `using`) |
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

**42 habits tidy will flag** (and what to write instead):

| 42 habit | Instead | Why |
|---|---|---|
| `int buf[64];` | `std::array<std::uint8_t, 64> buf{};` | Knows its size, bounds-checked with `.at()` |
| `malloc`/`free`, `new`/`delete` | Containers, `std::make_unique` | Memory is freed automatically, even on early returns (RAII) |
| `(int)x` | `static_cast<int>(x)` | Visible, searchable, and the compiler checks it |
| `NULL` | `nullptr` | Typed. `NULL` is just `0` |
| `int x;` (uninitialized) | `int x{0};` | Reading uninitialized memory is undefined behaviour |
| `printf` | `RCLCPP_INFO(...)` | Goes to ROS logs with timestamps and levels |
| `if (count)` on an integer | `if (count != 0)` | States the intent (pointers in `if (ptr)` are allowed) |
| `double d = …; int i = d;` | `static_cast<int>(std::lround(d))` | Silent truncation hides unit and rounding bugs |

**T1. Function size limits:** warn above **80 lines** or **6 parameters**. The 42 Norm was 25 lines.
- [ ] Keep 80 / 6
- [ ] Stricter, e.g. 50 / 5
- [ ] Looser

**T2. When warnings become errors.** Today everything is a warning (`WarningsAsErrors: ''`). Proposal: from **Sprint 2**, make `bugprone-*`, `cert-*`, `clang-analyzer-*` and `concurrency-*` fail CI, and add more groups as the code gets clean. *Why: warnings that never fail anything get ignored, but making everything an error on day one blocks learning.*
- [ ] Agree
- [ ] All errors now
- [ ] Other: ______________________

### 3.4 C++ practices the tools don't check

**P1. Language standard: C++17.** *Why: it's ROS 2 Jazzy's target standard, so every ROS header and example is tested with it.* C++20 works with Jazzy's GCC 13 (concepts, `std::span`, designated initializers) but isn't what the ROS examples use.
- [ ] C++17
- [ ] C++20

**P2. Real-time code path.** Inside the ros2_control `read()`/`write()` loop:
- No memory allocation (`new`, `push_back` growth, `std::string` building).
- No exceptions.
- No blocking longer than the serial I/O itself.
- Report failures with `return_type::ERROR`.

*Why: this loop runs at a fixed rate, and allocation or exceptions can stall it unpredictably. That's exactly the jitter M1 asks us to measure.*
- [ ] Agree
- [ ] Discuss further

**P3. Ownership:** no raw owning pointers. Use `std::unique_ptr` by default, `std::shared_ptr` only when ownership is really shared, and RAII wrappers for OS resources (the serial port's file descriptor is closed in a destructor).
- [ ] Agree

**P4. Units:** SI units (metres, radians, seconds) on every ROS interface (REP 103). Convert the arm's degrees and millimetres **only inside the driver**. *Why: a unit bug on a robot is a crash, not a wrong number on a screen.*
- [ ] Agree

**P5. Header guards:** `#pragma once`, or classic include guards (`POSITRONIC_HARDWARE__SERIAL_PORT_HPP_`)? ROS 2 uses guards, and its `cpplint` linter requires them if we keep it (see L1).
- [ ] Include guards (ROS style)
- [ ] `#pragma once` (shorter; needs `cpplint` disabled)

**P6. Comments:** explain **why**, not what. Doxygen `///` on public headers only.
- [ ] Agree
- [ ] Doxygen everywhere
- [ ] No Doxygen

**P7. Logging:** `RCLCPP_*` only. Never log every cycle inside the control loop; use the `_THROTTLE` variants. *Why: logging in a 50 Hz loop floods the console and costs time.*
- [ ] Agree

### 3.5 ROS linters in each package **[DISCUSS]**

Packages run `ament_lint_auto` in `make test`. Its default bundle has some tools that overlap with ours:

**L1. Linter set.** Proposal:
- Keep `cppcheck` (extra static analysis), `cpplint` (header guards, a few style rules), `lint_cmake` and `xmllint`.
- **Exclude `uncrustify`.** It's a second formatter with different rules, so it would fight clang-format.
- **Exclude `copyright`.** It demands a licence header in every file, and our `LICENSE` file already covers the repo.

- [ ] Agree
- [ ] Keep copyright headers
- [ ] Other: ______________________

### 3.6 Python

Python is used for launch files, tools, tests and, from M2, ML.

- **Python 3.12**, the version in the Jazzy image. *Why: code runs where it's tested.*
- **ruff** for linting *and* formatting (one tool instead of black plus a linter), with **99 columns** (ROS's Python convention).
- PEP 8 naming:
  - `snake_case` functions and variables.
  - `CamelCase` classes.
  - `UPPER_CASE` constants.
- Tests with **pytest**.
- Note: CI still runs Python 3.8 with black. Moving it to the team image is issue **#28**.

**Y1. Type hints:** required on public functions?
- [ ] Yes
- [ ] Encouraged, not required
