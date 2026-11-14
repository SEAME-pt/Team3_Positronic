# HIL runner on the Jetson

`.github/workflows/hil.yml` runs the tests of `src/positronic_hil` on the real arm, through a
self-hosted GitHub Actions runner on the Jetson. It runs only by hand, because the tests power the
servos and someone must check the bench first.

## How it works

1. Someone starts it: Actions → HIL → Run workflow → the branch to test.
2. The runner builds the Jetson image from that branch.
3. It stops `positronic-driver.service` if it is running.
4. It fails if `make nano-shell` is open or `/dev/ttyTHS1` is in use.
5. It runs the HIL tests on the arm. A skipped test fails the run.
6. It starts `positronic-driver.service` again and uploads the results as the artifact `hil-results`.

## Running the HIL

1. The arm powered, nothing within its reach, nobody in `make nano-shell`.
2. Actions → HIL → Run workflow → choose the branch.

## Recovery

| Problem | Fix |
| :--- | :--- |
| The run waits with "Waiting for a runner" | On the Jetson: `cd ~/actions-runner && sudo ./svc.sh start`. |
| A wrong label on the runner | Settings → Actions → Runners → `jetson` → Labels: untick it. |
| Register the runner again | `sudo ./svc.sh stop`, `sudo ./svc.sh uninstall`, `./config.sh remove --token <TOKEN>`, then follow Settings → Actions → Runners → New self-hosted runner with `--labels jetson,hil` and `sudo ./svc.sh install er`. |
| "Stop the driver service" fails | `er` needs `er ALL=(root) NOPASSWD: /usr/bin/systemctl stop positronic-driver, /usr/bin/systemctl start positronic-driver` in `/etc/sudoers.d/positronic-hil`. |
| "Check the arm is free" fails | Someone is in `make nano-shell`: ask them to exit. |
| `positronic-driver` stayed stopped after a run | `sudo systemctl start positronic-driver`. |

