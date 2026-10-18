# Guide: Git on Windows for this repository

**Version:** 1.0
**Status:** ACTIVE
**Updated:** 2026-10-17
**Applies to:** a Windows 10/11 laptop with the Arduino IDE; repository `github.com/ticohannan/rot3u-robots`

Type the commands in **Git Bash** (installed with Git). PowerShell works too, for every command here.

## A. One-time setup

1. Download Git for Windows from <https://git-scm.com/download/win> and run the installer. The default
   choices are right; in particular keep:
   - "Git from the command line and also from 3rd-party software";
   - "Checkout Windows-style, commit Unix-style line endings";
   - "Git Credential Manager" (signs you in to GitHub through your browser).
2. Open **Git Bash** from the Start menu.
3. Tell Git who you are. Use GitHub's private address so your e-mail is not published in the history.
   Find it at <https://github.com/settings/emails> under "Keep my email address private"; it looks like
   `12345678+ticohannan@users.noreply.github.com`.
   ```
   git config --global user.name "ticohannan"
   git config --global user.email "12345678+ticohannan@users.noreply.github.com"
   git config --global init.defaultBranch main
   ```
4. Check: `git config --global --list`.

## B. Get the repository onto the laptop

Pick **one** of these.

### B1. The code is already on GitHub (normal case)

1. Choose where it lives. The Arduino sketchbook folder is convenient:
   ```
   cd ~/Documents/Arduino
   git clone https://github.com/ticohannan/rot3u-robots.git
   cd rot3u-robots
   ```
2. The first time, a browser window asks you to sign in to GitHub. After that Windows remembers it.

### B2. You were given the repository as a zip (it has a `.git` folder inside)

1. Unzip it to `Documents\Arduino\rot3u-robots`, so that `Documents\Arduino\rot3u-robots\README.md`
   exists.
2. In Git Bash:
   ```
   cd ~/Documents/Arduino/rot3u-robots
   git status                        # "On branch main ... nothing to commit"
   git log --oneline                 # the history that came with it
   git remote -v                     # should show origin https://github.com/ticohannan/rot3u-robots.git
   ```
   If `git remote -v` shows nothing: `git remote add origin https://github.com/ticohannan/rot3u-robots.git`
3. Publish it to the empty GitHub repository:
   ```
   git push -u origin main
   ```
   Sign in through the browser when asked. Refresh the GitHub page: the files are there.

### B3. Starting a brand-new repository from your own files

```
cd ~/Documents/Arduino/my-project
git init
git add .
git commit -m "First version"
git remote add origin https://github.com/ticohannan/<new-repo>.git
git push -u origin main
```

## C. Open the sketches in the Arduino IDE

1. **File → Open**, then `Documents\Arduino\rot3u-robots\firmware\ik_demo\ik_demo.ino`.
2. The IDE shows the other files of the sketch folder as tabs: `robot_config.h` and `task_config.h`
   are the ones you edit; the driver headers are copies (edit them only in `firmware/common/`).
3. Board: **Arduino Mega or Mega 2560**. Upload as usual.
4. For a pin-driven robot (no PCA9685), install the **Servo** library (Library Manager) and uncomment
   `#define SERVO_DRIVER_PINS` in `robot_config.h`.

## D. Everyday use

| You want to | Type |
| --- | --- |
| Get the latest from GitHub (start of a session) | `git pull` |
| See what you changed | `git status` then `git diff` |
| Save a set of changes locally | `git add -A` then `git commit -m "Wrist recalibrated after horn change"` |
| Send them to GitHub | `git push` |
| See the history | `git log --oneline` |
| Throw away an uncommitted change to one file | `git restore firmware/ik_demo/task_config.h` |
| Look at an old version of a file | `git show HEAD~3:firmware/ik_demo/robot_config.h` |

Commit small, related changes together, with a message that says *why*.

## E. Trying something without risk: branches

```
git switch -c lash-test            # a new branch for the experiment
... edit, commit as usual ...
git push -u origin lash-test       # publish the branch
```

On GitHub, open a **Pull Request** from `lash-test` into `main`. The tests run automatically (the
**Actions** tab shows a green tick or a red cross). Merge when it is green. Back on the laptop:

```
git switch main
git pull
```

To abandon an experiment: `git switch main` and `git branch -D lash-test`.

## F. The change process, in Git terms

1. Branch. 2. Edit the design document in `docs/design/` first and commit it. 3. Change the code and
add the test; commit. 4. Push and open a Pull Request; wait for the tests. 5. Merge. Details:
`CONTRIBUTING.md`.

## G. Calibration data is code

A new calibration means a new `robot_config.h`. Commit it with a message saying what was measured, and
copy it to `configs/<robot>/` too, so the next assignment can start from it.

## H. Marking versions

```
git tag ik_demo-v1.0
git push --tags
```

## I. Running the tests on the laptop (optional)

GitHub runs them on every push, so this is optional.

1. Python with numpy (WinPython has both).
2. A `g++` compiler: install MSYS2 from <https://www.msys2.org>, then in the "MSYS2 UCRT64" window run
   `pacman -S mingw-w64-ucrt-x86_64-gcc`, and add `C:\msys64\ucrt64\bin` to your Windows PATH.
3. `python tests/run_tests.py`. The AVR build tests are skipped without avr-gcc; that is expected.

Not yet tried on Windows: if a host test fails there but passes on GitHub, the GitHub result is the
reference.

## J. If something goes wrong

| Message | Fix |
| --- | --- |
| `rejected ... fetch first` on push | Someone (or you, elsewhere) pushed first: `git pull`, then `git push` |
| `CONFLICT` after a pull | Open the file, keep the right lines, delete the `<<<<<<<`/`=======`/`>>>>>>>` markers, then `git add file` and `git commit` |
| `Permission denied` / `403` | Sign in again: `git credential-manager github login` |
| `LF will be replaced by CRLF` | Harmless: Git is converting line endings as configured |
