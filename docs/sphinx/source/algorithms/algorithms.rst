====================================
Algorithm Base Classes for PyRobot
====================================

The user should extend one of the base classes with customed implementations.

All algorithm base class should extend the algorithm abstract class:

.. autoclass:: pyrobot.algorithms.algorithm.Algorithm
        :members:
        :show-inheritance:
        :special-members:
        :exclude-members: __metaclass__, __weakref__

At this point, the following base classes are available:

.. toctree::
   :maxdepth: 2

   base_controller
   base_localizer
   base_planner
   camera_transform
   frame_transform
   kinematics
   motion_planner