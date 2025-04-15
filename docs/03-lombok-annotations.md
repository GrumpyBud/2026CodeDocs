# Lombok Annotations

## What is Lombok?
Lombok is a Java library that reduces boilerplate code by generating common methods and constructors using annotations like @Getter, @Setter, @Builder, @RequiredArgsConstructor, and more. It is widely used in advanced FRC codebases to keep code concise and maintainable.

### Why Use Lombok in FRC?
- **Less Boilerplate**: No need to manually write getters, setters, constructors, or builders.
- **Immutability**: Easily create immutable classes with final fields and @RequiredArgsConstructor.
- **Readability**: Focus on logic, not repetitive code.
- **Advanced Patterns**: Use @Builder for complex configuration/state objects, @ExtensionMethod for DSL-like code, and @Data for quick POJOs.

## Example from RobotCode2025Public
```java
@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
    ALGAE_L2_CORAL(...),
    ALGAE_L3_CORAL(...);
    // ...
    private final SuperstructureStateData value;
}
```

## How to Use in FRC
1. **@Getter/@Setter**: Add automatic property accessors to fields or classes.
2. **@RequiredArgsConstructor**: Generate a constructor for all final fields (great for dependency injection).
3. **@Builder**: Create builder APIs for complex objects.
4. **@Data**: Combine @Getter, @Setter, @EqualsAndHashCode, @ToString, and @RequiredArgsConstructor.
5. **@ExtensionMethod**: Add static utility methods as if they were instance methods.

## FRC Example: Subsystem with Lombok
```java
@Getter
@RequiredArgsConstructor
public class SwerveModule {
    private final int id;
    private final double angle;
}

// Usage:
SwerveModule module = new SwerveModule(1, 90.0);
int id = module.getId();
```

## Advanced Example: Builder for Configuration
```java
@Builder
public class ShooterConfig {
    private final double rpm;
    private final double angle;
}

ShooterConfig config = ShooterConfig.builder().rpm(5000).angle(45).build();
```

## Best Practices
- Use @Getter/@Setter for all fields unless you have a reason not to.
- Use @Builder for objects with many optional parameters.
- Use @RequiredArgsConstructor for dependency injection in subsystems.
- Use @Data for simple, immutable data containers.

## Anti-Patterns to Avoid
- Overusing @Data on complex classes (can expose too much API).
- Using @Setter on fields that should be immutable.
- Relying on Lombok for logic (use it for structure, not behavior).

## Migration Tips
- Replace manual getters/setters with Lombok annotations.
- Use @Builder for configuration/state objects.
- Use @RequiredArgsConstructor for subsystems with final dependencies.

## FRC-Specific Example: Enum with Lombok
```java
@Getter
@RequiredArgsConstructor
public enum IntakeState {
    INTAKING(1.0),
    OUTTAKING(-1.0),
    HOLDING(0.2);
    private final double voltage;
}
```

## References
- [Project Lombok](https://projectlombok.org/)
- [Lombok Features](https://projectlombok.org/features/all)
- [Effective Java: Immutability](https://www.oreilly.com/library/view/effective-java/9780134686097/ch04.html)
