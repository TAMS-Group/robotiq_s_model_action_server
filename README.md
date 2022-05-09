# robotiq_s_model_action_server

This repository provides a simple action server for robotiqs 3 finger gripper.
This supports simple open and close modes, depending on the branch.

---

## Usage

You can start the action server with the following command:

```bash
roslaunch robotiq_s_model_action_server robotiq_s_model_action_server.launch
```

To start with all required nodes:

```bash
roslaunch robotiq_s_model_action_server robotiq_s_model_gripper.launch
```

## Different grasp modes

The master branch provides a grasp with the open position in the wide mode and the closing position in the closed pinch mode. 
For the closing and opening movement the position of the fingers interpolate between these two positions.

The basic_full_grip branch provides a grasp with the open position in the opened basic mode and the close position
in the closed basic mode.
