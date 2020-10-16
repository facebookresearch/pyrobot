# franka_moveit

Some helper classes which interfaces moveit with the Franka Emika Panda robot.

### PandaMoveGroupInterface

- Provides interface to control and plan motions using MoveIt in ROS.
- Simple methods to plan and execute joint trajectories and cartesian path.
- Provides easy reset and environment definition functionalities (See ExtendedPlanningSceneInterface below).

### ExtendedPlanningSceneInterface

- Easily define scene for robot motion planning (MoveIt plans will avoid defined obstacles if possible).
