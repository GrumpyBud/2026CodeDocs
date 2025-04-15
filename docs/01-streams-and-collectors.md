# Java Streams and Collectors

## What are Streams and Collectors?
Java Streams provide a modern, functional approach to processing collections of data. Streams allow you to filter, map, reduce, and collect data in a concise and readable way. Collectors are used to gather the results of stream operations into collections like lists, sets, or maps, or to perform aggregations.

### Why Use Streams in FRC?
- **Conciseness**: Replace verbose for-loops with clear, declarative code.
- **Readability**: Express data transformations and filtering in a single statement.
- **Immutability**: Encourage functional, side-effect-free code.
- **Parallelism**: Easily process data in parallel if needed (rare in FRC, but possible).

## Example from RobotCode2025Public
```java
Set<CoralObjective> l1CoralObjectives =
    IntStream.rangeClosed(0, 11)
        .mapToObj(id -> new CoralObjective(id, ReefLevel.L1))
        .collect(Collectors.toSet());

List<Double> currents = motors.stream()
    .map(Motor::getCurrent)
    .collect(Collectors.toList());
```

## How to Use in FRC
1. **Filtering**: Select only relevant elements (e.g., active modules, sensors above a threshold).
2. **Mapping**: Transform objects (e.g., get sensor values from sensor objects).
3. **Reducing**: Aggregate data (e.g., sum, average, min/max).
4. **Grouping**: Organize data by a property (e.g., group modules by state).
5. **Collecting**: Gather results into lists, sets, or maps.

## FRC Example: Filtering and Mapping
```java
List<Double> highCurrents = motors.stream()
    .map(Motor::getCurrent)
    .filter(current -> current > 20.0)
    .collect(Collectors.toList());
```

## Advanced Example: Grouping and Partitioning
```java
Map<Boolean, List<Module>> byActive = modules.stream()
    .collect(Collectors.partitioningBy(Module::isActive));

Map<ReefLevel, List<CoralObjective>> byLevel = objectives.stream()
    .collect(Collectors.groupingBy(CoralObjective::reefLevel));
```

## Best Practices
- Use streams for data processing, not for side effects (avoid .forEach for mutating state).
- Prefer method references (e.g., Motor::getCurrent) for clarity.
- Use Collectors for grouping, partitioning, and aggregation.
- Avoid overusing streams for very simple or performance-critical code.

## Anti-Patterns to Avoid
- Using streams for everything (sometimes a for-loop is clearer or faster).
- Mutating external state inside streams.
- Deeply nested or unreadable stream chains.

## Migration Tips
- Refactor for-loops that filter/map/collect data into streams.
- Use streams for telemetry aggregation, dashboard data, and sensor processing.
- Use Collectors for grouping and summarizing data for logging or dashboards.

## FRC-Specific Example: Logging All Module Currents
```java
Logger.recordOutput("Drivetrain/ModuleCurrents",
    modules.stream().mapToDouble(Motor::getCurrent).toArray());
```

## References
- [Java Stream API](https://docs.oracle.com/javase/8/docs/api/java/util/stream/Stream.html)
- [Collectors](https://docs.oracle.com/javase/8/docs/api/java/util/stream/Collectors.html)
- [Effective Java: Streams](https://www.oreilly.com/library/view/effective-java/9780134686097/ch07.html)
