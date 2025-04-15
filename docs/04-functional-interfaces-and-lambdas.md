# Functional Interfaces and Lambdas

## What are Functional Interfaces and Lambdas?
Functional interfaces are Java interfaces with a single abstract method (SAM). Lambdas provide a concise syntax for implementing these interfaces without writing full anonymous classes. This functional programming approach enables more expressive, readable, and maintainable code - especially for callbacks, event handlers, and command creation.

### Core Java Functional Interfaces
- `Supplier<T>`: Provides a value of type T (no input, returns output)
- `Consumer<T>`: Accepts a value of type T (takes input, no return)
- `Function<T, R>`: Maps a value of type T to a value of type R (input → output)
- `Predicate<T>`: Tests a condition on a value of type T (returns boolean)
- `BooleanSupplier`: Specialized supplier that returns a boolean

### WPILib Functional Interface Usage
WPILib extensively uses functional interfaces for:
- Command creation and composition
- Triggers and button bindings
- Sensor value suppliers
- Robot state monitoring
- Telemetry and logging

## Why Use Functional Interfaces in FRC?
- **Concise Command Creation**: Build commands without creating separate classes
- **Dynamic Behaviors**: Change behaviors at runtime based on conditions
- **Deferred Execution**: Define behaviors without executing immediately
- **Improved Readability**: Express intent more clearly than anonymous classes
- **Testable Components**: Easily mock or swap behaviors for testing

## Example from RobotCode2025Public
```java
// Creating a command with a lambda
public Command algaeHold() {
    return runOnce(() -> setIntakeVoltage(.4));
}

// Using suppliers for dynamic values
new TrapezoidProfile(
    new TrapezoidProfile.Constraints(
        Units.degreesToRadians(maxVelocityDegPerSec.get()),
        Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
        
// Button binding with lambda
new Trigger(() -> m_controller.getRightBumper())
    .onTrue(m_exampleSubsystem.exampleMethodCommand());
```

## WPILib-Specific Functional Interfaces
- `CommandSupplier`: Returns a command (often used in factories)
- `BooleanEvent`: Used in trigger bindings to detect state changes

## How to Use in FRC

### For Commands
```java
// Simple action
Command intakeCommand = runOnce(() -> intake.setSpeed(1.0));

// Command with end condition
Command driveDistance = Commands.run(
    () -> drivetrain.drive(0.5, 0), 
    () -> drivetrain.getDistance() > 5.0,
    drivetrain
);

// Command factory method
public Command createAimCommand(Supplier<Pose2d> targetSupplier) {
    return run(() -> {
        Pose2d target = targetSupplier.get();
        double angle = calculateAngleTo(target);
        turret.setAngle(angle);
    });
}
```

### For Triggers and Buttons
```java
// Button press
new JoystickButton(driverController, Button.kA.value)
    .onTrue(intake.runCommand());
    
// Custom condition
new Trigger(() -> getAverageCurrent() > 10)
    .onTrue(Commands.runOnce(() -> limitPower()));
    
// Complex condition
new Trigger(() -> 
    robotState.hasGamePiece() && 
    robotState.isNearScoringPosition())
    .onTrue(autoScoreCommand);
```

### For Sensor and State Monitoring
```java
// Pass a supplier instead of a value
shooter.setTargetRPM(() -> dashboard.getSelectedRPM());

// On-the-fly calculation
limelight.setPipelineSupplier(() -> 
    isInAutoMode ? Pipelines.APRILTAG : Pipelines.REFLECTIVE);
```

## Advanced Techniques

### Method References
Instead of writing lambdas, use method references for cleaner code:

```java
// Instead of:
motors.forEach(motor -> motor.stop());

// Use:
motors.forEach(Motor::stop);

// Instead of:
new Trigger(() -> joystick.getRawButton(1))

// Use:
new Trigger(joystick::getRawButton)
```

### Combining Functional Interfaces
```java
// Combine predicates
Predicate<SwerveModule> isFaulted = module -> module.isFaulted();
Predicate<SwerveModule> isHot = module -> module.getTemperature() > 60;
Predicate<SwerveModule> needsAttention = isFaulted.or(isHot);

// Use with streams
List<SwerveModule> modulesNeedingAttention = modules.stream()
    .filter(needsAttention)
    .collect(Collectors.toList());
```

### Closures and Variable Capture
Lambdas can capture variables from their surrounding scope, but they must be effectively final:

```java
// This works:
double maxSpeed = calculateMaxSpeed();
Command driveCommand = run(() -> drivetrain.drive(maxSpeed, 0));

// This won't compile:
double speed = 0.5;
Command badCommand = run(() -> {
    speed += 0.1; // ERROR: Cannot modify captured variable
    drivetrain.drive(speed, 0);
});

// Solution: Use an AtomicReference or similar:
AtomicReference<Double> speedRef = new AtomicReference<>(0.5);
Command goodCommand = run(() -> {
    speedRef.updateAndGet(s -> s + 0.1); // OK
    drivetrain.drive(speedRef.get(), 0);
});
```

## Anti-Patterns to Avoid
- **Heavy Computation**: Don't put heavy computation in frequently called lambdas
- **Side Effects**: Avoid modifying external state within lambdas when possible
- **Overcomplicated Lambdas**: Break down complex lambdas into named methods
- **Ignoring Method References**: Use method references when cleaner than lambdas
- **Thread Safety Issues**: Be careful with shared state in multithreaded contexts

## Migration Tips for FRC Teams
1. Start with simple command lambdas to replace standalone command classes
2. Use suppliers for dashboard-configurable values
3. Replace anonymous inner classes with lambdas
4. Learn and use method references where appropriate
5. Use functional interfaces for strategy pattern implementation

## Real-World FRC Example: Dynamic Auto Routine Builder
```java
public Command createScoringSequence(Supplier<GamePiece> pieceSupplier) {
    return Commands.sequence(
        // Move arm based on the type of game piece
        Commands.runOnce(() -> {
            GamePiece piece = pieceSupplier.get();
            if (piece == GamePiece.CUBE) {
                arm.setPosition(ArmPositions.CUBE_SCORE);
            } else {
                arm.setPosition(ArmPositions.CONE_SCORE);
            }
        }),
        // Wait until arm is in position
        Commands.waitUntil(arm::isAtTarget),
        // Release the game piece
        Commands.runOnce(gripper::release),
        // Wait a moment
        Commands.waitSeconds(0.5),
        // Return to stowed position
        Commands.runOnce(() -> arm.setPosition(ArmPositions.STOWED))
    );
}
```

## References
- [Java Functional Interfaces (java.util.function)](https://docs.oracle.com/javase/8/docs/api/java/util/function/package-summary.html)
- [WPILib Command Framework](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [Method References](https://docs.oracle.com/javase/tutorial/java/javaOO/methodreferences.html)
- [Lambda Expressions](https://docs.oracle.com/javase/tutorial/java/javaOO/lambdaexpressions.html)
