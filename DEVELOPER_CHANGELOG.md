# Developer changelog (DimOS)

Brief log of non-trivial repo changes made during assisted sessions.

## 2026-04-02 — Learning guide

- Added `LEARNING_GUIDE.md` at repository root: reader-oriented map of architecture, execution flow, hardware targets, algorithms, IPC, configuration, full test file index (156 files), auto-generated `all_modules` registry excerpt, and an explicit note that RL training/reward/export for locomotion ONNX policies is **not** implemented in-tree (inference-only in `simulation/mujoco/policy.py`). No runtime behavior changed.

## 2026-04-02 — WebRTC + manipulation + drone deep dives

- Extended `LEARNING_GUIDE.md` with **§14** Phase 1 (Unitree WebRTC): `unitree_webrtc_connect` split, `UnitreeWebRTCConnection` connect/asyncio thread, `RTC_TOPIC` usage table, `move()` / `WIRELESS_CONTROLLER` mapping, `GO2Connection`/`G1Connection`/`UnitreeSkillContainer`, audio path, pickle shims, mermaid sequence.
- Added **§15** Manipulation: `ManipulationModule` config/state, `planning/factory.py` (Drake world, Jacobian/Drake IK, RRT-Connect), `ControlCoordinator` tick loop.
- Added **§16** Drone: `DroneConnectionModule`, `MavlinkConnection`/`pymavlink`, `DroneTrackingModule` + `DroneVisualServoingController`, `ros_command_queue` note.
