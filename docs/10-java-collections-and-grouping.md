# Java Collections and Grouping

## What are Java Collections?
Java Collections (List, Set, Map, etc.) are data structures for storing and organizing data. Grouping and partitioning with streams and collectors can simplify data processing.

## Why Use Them in FRC?
- Organize subsystem states, sensor data, and configuration.
- Use grouping to process and display telemetry or game state.

## Example from RobotCode2025Public
```java
var branchesForLevel =
    availableBranches.stream().collect(Collectors.groupingBy(CoralObjective::reefLevel));
```

## How to Use in FRC
- Use Map to associate IDs with subsystems or sensors.
- Use groupingBy to organize data for dashboards or analysis.

## FRC Example
```java
Map<Integer, MotorController> motorMap = new HashMap<>();
List<Double> currents = motors.stream().map(Motor::getCurrent).toList();
```

## References
- [Java Collections Framework](https://docs.oracle.com/javase/8/docs/technotes/guides/collections/overview.html)
