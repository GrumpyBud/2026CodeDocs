# Extension Methods

## What are Extension Methods?
Extension methods allow you to add new methods to existing types without modifying their source code. In Java, this is commonly achieved using static utility methods or, with Lombok, the @ExtensionMethod annotation.

## Why Use Them in FRC?
- Add custom helpers to WPILib or vendor classes (e.g., Pose2d, Rotation2d).
- Make code more readable and expressive.
- Reduce repeated utility code.

## Example from RobotCode2025Public
```java
@ExtensionMethod({GeomUtil.class})
public class RobotState {
    // ...
}
```

## How to Use in FRC
- Create a utility class with static methods.
- Use @ExtensionMethod to make those methods available as if they were part of the class.

## FRC Example
```java
@ExtensionMethod({MyMathUtil.class})
public class MySubsystem {
    // Now you can call myCustomClamp() as if it were a method on Double
}
```

## References
- [Lombok @ExtensionMethod](https://projectlombok.org/features/experimental/ExtensionMethod)
