# README Bench Manual Visuals Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add four accurate, polished SVG plates to the README in the approved B1 Bench Manual style.

**Architecture:** Keep every graphic as a standalone SVG under `docs/assets/readme/`, with an opaque
background and shared visual tokens repeated locally so GitHub needs no external CSS or fonts. Keep
the protocol tables and commands as Markdown; the illustrations introduce and summarize them.

**Tech Stack:** SVG 1.1, Markdown, PowerShell/XML validation, ImageMagick or browser rendering, ROS 2 Humble/colcon

---

### Task 1: Create the project hero

**Files:**
- Create: `docs/assets/readme/hero-bench-manual.svg`

- [ ] **Step 1: Build a 1440 x 640 SVG with the shared visual tokens**

Use an opaque `#F5F3EE` canvas, `#222B2E` primary text, `#197C7A` signal paths,
`#D65F4A` safety accents, and `#C9CECA` construction lines. Use only the system font stack
`Inter, ui-sans-serif, system-ui, -apple-system, Segoe UI, sans-serif`.

- [ ] **Step 2: Add verified project content**

Render the exact title `ros2-control-vcan-motor-demo`, subtitle
`A differential-drive control loop you can run without a motor controller`, a simplified two-wheel
robot connected to a CAN trace, and the four facts `42 TESTS`, `2 VIRTUAL MOTORS`, `6 CAN IDS`, and
`0 PHYSICAL HARDWARE`.

- [ ] **Step 3: Validate the SVG as XML**

Run:

```powershell
[xml](Get-Content -Raw docs/assets/readme/hero-bench-manual.svg) | Out-Null
```

Expected: command exits successfully without an XML parser error.

- [ ] **Step 4: Commit the hero**

```bash
git add docs/assets/readme/hero-bench-manual.svg
git commit -m "docs: add bench manual README hero"
```

### Task 2: Create the control-loop plate

**Files:**
- Create: `docs/assets/readme/control-loop.svg`

- [ ] **Step 1: Draw the real left-to-right control path**

Use a 1440 x 760 canvas and these nodes in order:

```text
/cmd_vel -> diff_drive_controller -> CanMotorHardware::write() -> vcan0
vcan0 -> virtual_motor_node -> ACK + encoder feedback -> CanMotorHardware::read()
CanMotorHardware::read() -> /joint_states + /odom
```

Show command IDs `0x101 / 0x102`, feedback IDs `0x181 / 0x182`, and ACK IDs
`0x281 / 0x282` at the bus boundary. Distinguish commands from feedback with solid and dashed teal
paths, and keep CAN as a single shared bus rather than a ROS topic bridge.

- [ ] **Step 2: Validate labels against source**

Run:

```powershell
rg -n "command_id|feedback_id|ack_id" src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_protocol.hpp
```

Expected: bases are `0x100`, `0x180`, and `0x280`, producing the six labels used in the plate for
node IDs 1 and 2.

- [ ] **Step 3: Validate XML and commit**

```powershell
[xml](Get-Content -Raw docs/assets/readme/control-loop.svg) | Out-Null
git add docs/assets/readme/control-loop.svg
git commit -m "docs: illustrate the vcan control loop"
```

### Task 3: Create the safety and protocol plates

**Files:**
- Create: `docs/assets/readme/safety-path.svg`
- Create: `docs/assets/readme/can-frame-layout.svg`

- [ ] **Step 1: Draw the safety path**

Use a 1440 x 720 canvas. Put `NORMAL CONTROL` on the teal path and group these injected conditions
on the coral path: `COMMAND LOSS`, `FEEDBACK LOSS`, `DELAY`, `MALFORMED DLC 7`, and `CAN ERROR
FRAME`. Route them through the actual checks `ACK TIMEOUT 200 ms`, `FEEDBACK TIMEOUT 500 ms`, and
`MOTOR WATCHDOG 200 ms`, ending at `DISABLED ZERO COMMANDS` and `SAFE STOP`.

- [ ] **Step 2: Draw the CAN byte layout**

Use a 1440 x 880 canvas with three eight-byte rows:

```text
COMMAND  0x101 / 0x102  [SEQ][FLAGS][VELOCITY i16 LE][WATCHDOG u16 LE][RESERVED 0][RESERVED 0]
FEEDBACK 0x181 / 0x182  [SEQ][STATUS][VELOCITY i16 LE][ENCODER COUNT i32 LE]
ACK      0x281 / 0x282  [SEQ][RESULT][RESERVED 0 x6]
```

Label all frames `CLASSIC CAN / 11-BIT ID / DLC 8 / LITTLE-ENDIAN`. Use proportional byte widths
and group multibyte fields without hiding their byte boundaries.

- [ ] **Step 3: Validate both files and commit**

```powershell
[xml](Get-Content -Raw docs/assets/readme/safety-path.svg) | Out-Null
[xml](Get-Content -Raw docs/assets/readme/can-frame-layout.svg) | Out-Null
git add docs/assets/readme/safety-path.svg docs/assets/readme/can-frame-layout.svg
git commit -m "docs: add CAN safety and frame diagrams"
```

### Task 4: Integrate the graphics into the README

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Add the hero under the badges**

Insert:

```markdown
![Bench manual view of the ros2_control vcan virtual motor demo](docs/assets/readme/hero-bench-manual.svg)
```

- [ ] **Step 2: Replace Mermaid with the control-loop plate**

Replace the current `flowchart LR` block with:

```markdown
![Control path from cmd_vel through ros2_control and vcan to two virtual motors](docs/assets/readme/control-loop.svg)
```

- [ ] **Step 3: Place the protocol and safety plates**

Insert the CAN frame image immediately after the `## CAN protocol` introduction and the safety
image immediately after the first fault-injection paragraph. Keep every existing table and command.

- [ ] **Step 4: Check Markdown references and commit**

Run:

```powershell
rg -n "docs/assets/readme|```mermaid" README.md
```

Expected: four asset references and no Mermaid block.

```bash
git add README.md
git commit -m "docs: integrate bench manual visuals"
```

### Task 5: Render and verify the finished README

**Files:**
- Verify: `docs/assets/readme/*.svg`
- Verify: `README.md`

- [ ] **Step 1: Render each SVG to PNG for visual inspection**

Use ImageMagick when available:

```powershell
Get-ChildItem docs/assets/readme/*.svg | ForEach-Object { magick $_.FullName ($_.FullName + '.png') }
```

If ImageMagick is unavailable, open the SVG files in the browser and take desktop and narrow-width
screenshots. Inspect each at full size and README scale for clipped or overlapping text.

- [ ] **Step 2: Check repository text and whitespace**

Run:

```bash
git diff master...HEAD --check
git status --short
```

Expected: no whitespace errors and no untracked render artifacts.

- [ ] **Step 3: Run the existing ROS test suite in WSL**

Run:

```bash
wsl -d Ubuntu-22.04 -- bash -lc 'cd "/mnt/c/Users/qucha/Documents/New project" && source /opt/ros/humble/setup.bash && colcon build --packages-select vcan_diffbot_demo && source install/setup.bash && colcon test --packages-select vcan_diffbot_demo && colcon test-result --verbose'
```

Expected: `42 tests, 0 errors, 0 failures, 0 skipped`.

- [ ] **Step 4: Push the feature branch**

```bash
git push -u origin feat/readme-bench-manual
```
