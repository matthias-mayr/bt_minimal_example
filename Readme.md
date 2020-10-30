# Minimal example for issue #240 in the C++ BT Library

Details are to be found [here](https://github.com/BehaviorTree/BehaviorTree.CPP/issues/240).

## Compile this code

This is a `catkin` package. Clone it into a catkin workspace, fullfill the dependencies and build it.

## Run this code

After building the code, it's best practice to source the workspace again with 
```
source ~/catkin_ws/devel/setup.bash
```
Run the example with
```
rosrun bt_minimal_example bt_minimal_example
```

## Expected output
Before every tick, the expected action is announced. The action after the tick should be consistent with the announced action.
