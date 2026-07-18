# README Bench Manual Visual Design

## Goal

Give the repository a recognizable first screen and explain the demo faster without turning the
README into a product landing page. The result should look like a well-kept open-source robotics
manual: technical, calm, and specific to this project.

## Visual Direction

Use the approved **B1 Bench Manual** direction. The base is warm white with graphite text, muted
teal for signals and control flow, and a small coral accent for faults or safety stops. Layouts use
fine rules, grid alignment, compact labels, and restrained type sizes. There are no gradients,
glowing effects, fake screenshots, invented hardware, or generic circuit decoration.

The graphics are deterministic SVG files stored in `docs/assets/readme/`. They use common system
font fallbacks and remain readable on both GitHub's light and dark page backgrounds by carrying
their own opaque canvas.

## Assets

### Hero banner

The banner leads with `ros2-control-vcan-motor-demo`, a short plain-language description, and a
simple differential-drive/CAN line drawing. A compact data strip shows only verified facts:
`42 tests`, `2 virtual motors`, `6 CAN IDs`, and `0 physical hardware`.

### Control loop

A left-to-right technical plate shows the real loop:

`cmd_vel` -> `diff_drive_controller` -> `CanMotorHardware::write()` -> `vcan0` ->
`virtual_motor_node` -> ACK/encoder frames -> `CanMotorHardware::read()` -> odometry and joint
state. Command CAN IDs `0x101/0x102` are visible at the bus boundary.

### Safety and fault path

A compact state-oriented diagram separates the normal path from injected loss, delay, malformed
frames, and CAN error frames. It shows ACK timeout, command timeout, motor watchdog, and safe zero
velocity as the final protective action. The diagram describes existing behavior only.

### CAN frame layout

A byte-level plate documents command, ACK, and encoder frame families with their actual CAN IDs,
DLC, endianness, sequence field, and payload meaning. Values are taken from the implementation and
the README protocol tables rather than reconstructed from memory.

## README Placement

The hero sits directly below the title and badges. The control-loop image replaces the current
Mermaid diagram so GitHub renders the same composition everywhere. The safety image appears beside
the existing safety/fault documentation, and the frame layout appears before the CAN protocol
tables. Tables and commands remain in text for searchability and accessibility.

Each image gets concise alt text. Links use repository-relative paths so they render on GitHub and
in local Markdown previews.

## Validation

- Cross-check every label, CAN ID, and stated count against source or test output.
- Render all SVGs and inspect them at desktop and narrow README widths.
- Confirm that no text clips, overlaps, or becomes unreadable in the scaled images.
- Run `git diff --check` and the existing project test suite before the branch is pushed.
- Review the final README on GitHub after push to catch Markdown rendering differences.

## Scope

This change adds four SVG assets and adjusts README composition and nearby prose. It does not add a
website, generated photography, a new documentation framework, or CI badges that the repository
does not currently support.
