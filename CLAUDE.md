# CLAUDE.md
This file provides guidance to Claude Code when working with this repository.

## Overview
A GitHub repository of ROS 2 packages containing reusable BehaviorTree.CPP action nodes for robotics applications.

Packages:
- `ros_cpp_behavior_util/`

## Architecture
Each package uses the `<package_name>` namespace, with headers in `include/<package_name>/` and implementations in `src/<package_name>/`.

**Shared pattern across all nodes:**
- The `rclcpp::Node::SharedPtr` is provided via the blackboard key `"node"`, not as an input port
- Both files begin with the author header: `////...// Author : Crasun Jans`
- All member variables and `static constexpr` constants use `snake_case_` (trailing underscore for private members, e.g. `node_`)
- All member functions use `camelCase()`
- Header guards use the form `<FILENAME>_HPP_`
- Put all includes in the header; implementation files only include their corresponding header
- Default to `StatefulActionNode` over `SyncActionNode` unless explicitly stated otherwise

### StatefulActionNode lifecycle
- `onStart()` — called once; reads ports, allocates resources, dispatches async work; returns `RUNNING` or `FAILURE`
- `onRunning()` — called every tick while `RUNNING`; checks progress; returns `RUNNING`, `SUCCESS`, or `FAILURE`
- `onHalted()` — called on external halt; cancels pending async operations and releases resources

### Port conventions
- Required input ports: use `getInput<T>()` and throw `BT::RuntimeError` on missing/invalid values
- Complex types use `std::shared_ptr<T>` to avoid expensive copying (e.g. `geometry_msgs::msg::PoseStamped`)

## Adding a new node
1. `include/<package_name>/<n>.hpp` — declare the class; document blackboard keys and ports in a Doxygen comment
2. `src/<package_name>/<n>.cpp` — implement the lifecycle methods and any helper functions; ensure proper error handling and resource management.
3. Register the new `.cpp` in `CMakeLists.txt` under `add_library()`
4. Add any new ROS dependencies to `ament_target_dependencies()`, `ament_export_dependencies()`, and `package.xml`
5. Add any new non-ROS dependencies to `target_link_libraries()` 
