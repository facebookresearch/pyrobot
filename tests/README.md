# Testing

## Algorithms

### Class-level testing
**Testing abstract class implementation**
- test for class-level constructor dependencies (robot instance)
- test for proper launching of rosservice and namespacing of rosparams

**Test shared by all implementation of the class**
- test for output type of each class function

### Instance-level testing
**For our implementation:**
- we provide behavioral test for each member function given input/output

**For user implementations:**
- we provide a test function that they can call using their customized input/output

## World

### Roslauncher
- proper launch/kill/namespacing behavior (both integrated test and mock rosnodes)

### Robot instance
- proper launch/kill/namespacing behavior in sim
- check for publishing and subscribing using mock sensors/actuators

### Algorithms
- graph construction (algorithm dependencies + shared dependencies)
- overload ros namespace (override node and informative warning)
- accessibility (algorithms registered under world vs. registered within algorithm only)

### Make functions
**make_robot / make_algorithm / make_sensor**
- check make instance type
- check output behavior: type, format, timestamp / frequency
- check interaction with world (when it should be accessible in world, etc.)