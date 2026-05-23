# AGENTS.md

## Environment

Project stack:

- ROS2 Jazzy
- Ubuntu 24.04
- Gazebo Sim 8 (`gz sim`)
- TurtleBot4
- Python-only ROS nodes (`rclpy`)
- `ros_gz_sim`
- `ros_gz_bridge`
- Nav2 ecosystem available
- Standard colcon workspace layout

Workspace structure:

```text
workspace/
├── src/
├── build/
├── install/
└── log/
```

Custom helper `.sh` scripts may exist and should be reused when appropriate.

---

# Core Principles

## Keep Changes Minimal

Prefer the smallest safe change that solves the problem.

Avoid:
- unnecessary refactors
- architecture rewrites
- introducing abstractions early
- overengineering
- async complexity
- threading unless explicitly required

The code only needs to work reliably and be easy to debug.

---

## Prefer Small Focused Nodes

Prefer:
- small single-purpose ROS nodes
- simple publishers/subscribers
- explicit behavior

Avoid:
- giant multifunction nodes
- hidden state
- deeply coupled logic

---

## Human Controls Simulation

The human operator launches simulation manually.

Agents must NOT:
- invent launch workflows
- restructure simulation startup
- assume launch topology
- modify Gazebo orchestration unless explicitly requested

---

# Build Rules

Default to package-selective builds:

```bash
colcon build --packages-select <pkg> --symlink-install
```

Avoid rebuilding the entire workspace unless necessary.

After builds, assume:

```bash
source install/setup.bash
```

---

# ROS2 Python Rules

## Logging

Use ROS logging only:

```python
self.get_logger().info("message")
```

Avoid:
- `print()`
- excessive spam logging

Throttle repetitive logs when appropriate.

---

## Parameters

Prefer ROS parameters over hardcoded constants.

Declare parameters explicitly.

Avoid:
- hidden magic numbers
- undeclared parameters
- unnecessary parameter proliferation

---

## Concurrency

Strongly prefer:
- timers
- callbacks
- executors

Avoid:
- Python threading
- unnecessary async patterns
- complex synchronization

Keep execution deterministic and easy to debug.

---

# TF and Simulation Rules

Assume simulation uses simulated time.

Preserve:
- `use_sim_time`
- `/clock`
- existing TF structure

Agents must NEVER:
- invent TF frames
- silently rename frames
- introduce new root frames
- add static transform hacks casually

Always verify TF assumptions before modifying navigation behavior.

---

# Navigation and Gazebo Rules

Project uses:
- Gazebo Sim (`gz sim`)
- `ros_gz_sim`
- `ros_gz_bridge`
- Nav2-compatible workflows

Do NOT:
- mix Gazebo Classic APIs with Gazebo Sim APIs
- introduce ROS1 syntax/examples
- assume topics/services without verification

Preserve existing:
- topic names
- QoS behavior
- Nav2 interfaces
- launch structure

---

# Safe Editing Policy

Before editing:

1. Understand the existing package structure
2. Prefer modifying existing nodes over creating new packages
3. Avoid touching launch files unless necessary
4. Avoid changing interfaces silently

Never:
- rename topics casually
- rename frames casually
- break public interfaces
- hardcode absolute paths
- modify robot description files unless explicitly requested

---

# Verification Policy

Never claim code works without explaining how to verify it.

After changes, provide:
- expected topics
- expected node behavior
- verification commands
- TF checks when relevant

Useful verification commands include:

```bash
ros2 topic list
ros2 topic echo <topic>
ros2 topic hz <topic>
ros2 node list
ros2 param list
ros2 action list
ros2 service list
```

For TF debugging:

```bash
ros2 run tf2_tools view_frames
```

Always check:
- `/clock`
- TF validity
- node names
- topic existence
- lifecycle state when using Nav2

---

# Anti-Hallucination Rules

Agents must NEVER:
- fabricate APIs
- fabricate topic names
- fabricate TF frames
- fabricate launch structure
- assume message types
- assume QoS settings

If uncertain:
- ask questions
- inspect existing code
- make conservative assumptions
- avoid speculative edits

---

# Code Style

Prefer:
- readable code
- explicit logic
- short functions
- straightforward control flow

Avoid:
- premature optimization
- metaprogramming
- unnecessary abstraction layers
- cleverness over clarity

Simple and reliable is preferred.

---

# RViz Rules

Do not overwrite RViz configurations unless explicitly requested.

Preserve existing visualization workflows and layouts.

---

# Preferred Workflow

Agents should follow this workflow:

1. Analyze existing structure
2. Propose minimal plan
3. Make incremental edits
4. Explain changes clearly
5. Provide verification steps

Step-by-step changes are preferred over large rewrites.

# Key repo locations
- src/dis_tutorial7 is the robotic arm camera package
- custom implementations belong to src/task1 unless specified otherwise

