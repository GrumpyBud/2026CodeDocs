# Use of Optional

## What is Optional?
`Optional<T>` is a container object which may or may not contain a non-null value. It helps avoid null pointer exceptions and makes APIs more expressive about the possibility of missing values.

## Why Use It in FRC?
- Makes it clear when a value might be absent (e.g., sensor data, subsystem state).
- Encourages safe handling of missing or unavailable data.
- Reduces bugs from unchecked nulls.

## Example from RobotCode2025Public
```java
public Optional<CoralObjective> getCoralObjective(ReefLevel level) {
    // ...
}
```

## How to Use in FRC
- Return `Optional<T>` from methods that may not always provide a value.
- Use `ifPresent`, `orElse`, and `map` to handle values safely.

## FRC Example
```java
Optional<Double> getSensorReading() {
    if (sensor.isConnected()) return Optional.of(sensor.getValue());
    else return Optional.empty();
}
```

## References
- [Optional (Java SE 8)](https://docs.oracle.com/javase/8/docs/api/java/util/Optional.html)
