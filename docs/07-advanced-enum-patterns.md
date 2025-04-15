# Advanced Enum Patterns

## What are Advanced Enum Patterns?
Java enums are much more powerful than simple lists of constants. They can have fields, methods, constructors, and even implement interfaces. This allows you to encapsulate both data and behavior for each enum constant, making your code more robust, readable, and maintainable.

### Why Use Advanced Enums in FRC?
- **State Machines**: Represent subsystem states (e.g., elevator positions, intake modes) with associated data and logic.
- **Configuration**: Store configuration values or strategies for each mode.
- **Behavior Encapsulation**: Attach methods to enum constants for state-specific behavior.
- **Type Safety**: Prevent invalid states and reduce bugs compared to using raw ints or strings.

## Example from RobotCode2025Public
```java
@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
    ALGAE_L2_CORAL(SuperstructureStateData.builder().pose(...).build()),
    ALGAE_L3_CORAL(SuperstructureStateData.builder().pose(...).build());
    // ...
    private final SuperstructureStateData value;
}
```

## How to Use in FRC
1. **Add Fields**: Store configuration or state data for each constant.
2. **Add Methods**: Implement logic that varies by state (e.g., isScoring(), getTargetHeight()).
3. **Implement Interfaces**: Allow enums to be used polymorphically (e.g., as a CommandSupplier).
4. **Use in Switch Statements**: Drive subsystem logic based on enum state.

## FRC Example: Intake Goals
```java
enum IntakeGoal {
    INTAKE(1.0),
    OUTTAKE(-1.0),
    HOLD(0.2);
    private final double voltage;
    IntakeGoal(double voltage) { this.voltage = voltage; }
    public double getVoltage() { return voltage; }
}

// Usage:
intakeMotor.set(intakeGoal.getVoltage());
```

## Advanced Example: Enum with Methods and Interface
```java
public interface StateBehavior { void execute(); }

public enum ElevatorState implements StateBehavior {
    BOTTOM { public void execute() { moveToBottom(); } },
    TOP { public void execute() { moveToTop(); } };
}
```

## Best Practices
- Use enums for all subsystem and robot states.
- Attach relevant data and logic to each constant.
- Avoid using raw ints or strings for state.
- Use @Getter/@RequiredArgsConstructor (Lombok) to reduce boilerplate.

## Anti-Patterns to Avoid
- Using enums as mere constants (add fields/methods for power!)
- Duplicating logic in switch statements instead of using enum methods
- Using magic numbers or strings for state

## Migration Tips
- Refactor int/string states to enums.
- Move state-specific logic into enum methods.
- Use enums in command factories and state machines.

## FRC-Specific Example: Autonomous Modes
```java
public enum AutoMode {
    FOUR_PIECE("Four Piece", () -> new FourPieceAuto()),
    TWO_PIECE("Two Piece", () -> new TwoPieceAuto());
    private final String name;
    private final Supplier<Command> commandSupplier;
    AutoMode(String name, Supplier<Command> commandSupplier) {
        this.name = name;
        this.commandSupplier = commandSupplier;
    }
    public Command getCommand() { return commandSupplier.get(); }
}
```

## References
- [Java Enum Types](https://docs.oracle.com/javase/tutorial/java/javaOO/enum.html)
- [Effective Java: Enums and Annotations](https://www.oreilly.com/library/view/effective-java/9780134686097/ch06.html)
- [Lombok @Getter](https://projectlombok.org/features/Getter)
