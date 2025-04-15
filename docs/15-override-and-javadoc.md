# Use of @Override and JavaDoc

## What is @Override and JavaDoc?
The `@Override` annotation tells the compiler that a method is intended to override a method in a superclass or interface. JavaDoc is a documentation system using specially formatted comments to generate API documentation. Both are essential tools for writing maintainable, professional-quality FRC code.

### @Override Annotation
`@Override` serves several important purposes:
- **Compile-time Error Checking**: If the method doesn't actually override anything, the compiler will generate an error
- **Code Clarity**: Clearly indicates the intention to override a method
- **Maintenance Safety**: Protects against superclass method signature changes

### JavaDoc Documentation
JavaDoc comments are special multi-line comments that start with `/**` and provide structured documentation for classes, interfaces, methods, and fields. They can include:
- General descriptions
- Parameter documentation (`@param`)
- Return value documentation (`@return`) 
- Exception documentation (`@throws`)
- Links to other elements (`{@link}`)
- Deprecation notices (`@deprecated`)

## Why Use Them in FRC?
- **Team Collaboration**: FRC teams have many programmers of varying experience levels and often high turnover
- **Future Maintainability**: Next year's programming team will thank you for clear documentation
- **Error Prevention**: `@Override` catches errors at compile-time that would be difficult to debug later
- **Learning Aid**: Well-documented code helps new team members understand the codebase
- **Readability**: Makes code intent clearer for everyone

## Example from RobotCode2025Public
```java
/**
 * Returns the current AprilTag layout type.
 */
@Override
public AprilTagLayoutType getSelectedAprilTagLayout() {
    if (aprilTagsReef.getAsBoolean()) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return FieldConstants.AprilTagLayoutType.BLUE_REEF;
        } else {
            return FieldConstants.AprilTagLayoutType.RED_REEF;
        }
    } else if (aprilTagNone.getAsBoolean()) {
        return FieldConstants.AprilTagLayoutType.NONE;
    } else {
        return FieldConstants.defaultAprilTagType;
    }
}
```

## How to Use in FRC

### @Override Best Practices
1. **Always Use It**: Apply `@Override` whenever overriding a method from a superclass or interface
2. **WPILib Robot Methods**: Always use it with robot lifecycle methods like `robotInit()`, `teleopPeriodic()`, etc.
3. **Command Overrides**: Always use it when overriding Command methods like `initialize()`, `execute()`, and `end()`
4. **Subsystem Overrides**: Always use it when overriding `periodic()` in subsystems

### JavaDoc Best Practices
1. **Document All Public APIs**: Write JavaDoc for all public classes, methods, and constants
2. **Include Purpose and Usage**: Explain what the method does, not just what it is
3. **Document Parameters**: Use `@param` tags to describe what each parameter means
4. **Document Return Values**: Use `@return` to explain what the method returns
5. **Document Units**: Always specify units for numerical values (e.g., meters, seconds, degrees)
6. **Include Examples**: For complex methods, include usage examples
7. **Note Side Effects**: Document any side effects the method might have

## FRC Example: A Well-Documented Subsystem

```java
/**
 * Elevator subsystem that manages vertical movement of the robot's scoring mechanism.
 * Uses a closed-loop control system to position the elevator at specified heights.
 */
public class Elevator extends SubsystemBase {
    
    private final TalonFX motor;
    private final DigitalInput limitSwitch;
    
    /**
     * Creates a new Elevator subsystem.
     * 
     * @param motorID The CAN ID of the elevator Falcon motor
     * @param limitSwitchPort The DIO port of the lower limit switch
     */
    public Elevator(int motorID, int limitSwitchPort) {
        motor = new TalonFX(motorID);
        limitSwitch = new DigitalInput(limitSwitchPort);
        
        // Configure motor settings
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
    }
    
    /**
     * Returns the current elevator height in meters from the bottom position.
     * 
     * @return The height in meters
     */
    public double getHeight() {
        return motor.getSelectedSensorPosition() * Constants.ELEVATOR_METERS_PER_TICK;
    }
    
    /**
     * Sets the target height for the elevator.
     * 
     * @param heightMeters The target height in meters from the fully retracted position
     * @throws IllegalArgumentException If the target height is outside the valid range
     */
    public void setTargetHeight(double heightMeters) {
        if (heightMeters < 0 || heightMeters > Constants.MAX_ELEVATOR_HEIGHT_METERS) {
            throw new IllegalArgumentException("Target height out of range: " + heightMeters);
        }
        
        double targetTicks = heightMeters / Constants.ELEVATOR_METERS_PER_TICK;
        motor.set(TalonFXControlMode.MotionMagic, targetTicks);
    }
    
    /**
     * Periodically updates the elevator control logic.
     * Handles limit switch detection and safety features.
     */
    @Override
    public void periodic() {
        // If limit switch is pressed, reset encoder position
        if (limitSwitch.get()) {
            motor.setSelectedSensorPosition(0);
        }
        
        // Log elevator state
        Logger.recordOutput("Elevator/HeightMeters", getHeight());
        Logger.recordOutput("Elevator/AtLowerLimit", limitSwitch.get());
    }
}
```

## Anti-Patterns to Avoid

1. **Empty JavaDoc**: 
   ```java
   /** */
   public void setSpeed(double speed) { ... }
   ```

2. **Redundant Documentation**:
   ```java
   /** Sets the speed */
   public void setSpeed(double speed) { ... }
   ```

3. **Missing Parameter Documentation**:
   ```java
   /** Sets the shooter to a specified velocity */
   public void setVelocity(double velocity) { ... } // What are the units? Valid ranges?
   ```

4. **Missing @Override**:
   ```java
   // Missing @Override - might not actually override as expected
   public void end(boolean interrupted) { ... }
   ```

## Migration Tips for FRC Teams

1. **Start with critical methods**: Begin by documenting the most important and complex subsystems
2. **Add @Override consistently**: Do a global search for methods that should have @Override
3. **Document as you code**: Write documentation while writing the code, not after
4. **Use an IDE that supports JavaDoc**: Most IDEs provide auto-completion for JavaDoc
5. **Review in code reviews**: Make JavaDoc and @Override part of your team's code review process

## References
- [Java @Override](https://docs.oracle.com/javase/tutorial/java/IandI/override.html)
- [How to Write JavaDoc](https://www.oracle.com/technical-resources/articles/java/javadoc-tool.html)
- [Google Java Style Guide - JavaDoc](https://google.github.io/styleguide/javaguide.html#s7-javadoc)
- [WPILib Documentation Style](https://docs.wpilib.org/en/stable/docs/software/wpilib-overview/index.html)
