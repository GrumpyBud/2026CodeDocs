# AutoLog and Logging Patterns

## What is AutoLog and Why is Logging Important?
AutoLog is a feature from AdvantageKit that automates the process of logging data for telemetry, debugging, and analysis. Logging is essential in FRC for understanding robot behavior, diagnosing issues, and improving performance both during development and at competitions. Good logging practices can be the difference between quickly solving a problem and spending hours guessing.

### Types of Logging in FRC
- **Automatic Logging**: Using annotations like `@AutoLogOutput` to automatically log fields.
- **Manual Logging**: Using `Logger.recordOutput` to log custom values or events.
- **Bulk Logging**: Registering entire objects with `AutoLogOutputManager` for comprehensive state capture.
- **Event Logging**: Logging discrete events (e.g., command start/stop, errors, state transitions).

## Why Use AutoLog and Logging Patterns in FRC?
- **Debugging**: Quickly identify the root cause of issues by reviewing logs.
- **Performance Analysis**: Analyze cycle times, subsystem response, and match performance.
- **Telemetry**: Visualize robot state in real time or post-match using tools like AdvantageScope.
- **Documentation**: Logs serve as a record of what happened during a match or test.
- **Collaboration**: Share logs with mentors or other teams for remote troubleshooting.

## Example from RobotCode2025Public
```java
@AutoLogOutput
private Rotation2d leftAlgaeObservation = Rotation2d.kZero;

AutoLogOutputManager.addObject(RobotState.getInstance());
Logger.recordOutput("Intake/No Coral", (m_colorSensor.getProximity() <= 120));
```

## How to Use in FRC
1. **Automatic Field Logging**: Annotate fields with `@AutoLogOutput` to have them logged automatically every cycle.
2. **Manual Logging**: Use `Logger.recordOutput("Path/To/Value", value)` for custom or computed values.
3. **Bulk Logging**: Register objects with `AutoLogOutputManager.addObject()` to log all annotated fields in that object.
4. **Event Logging**: Log events such as command start/stop, errors, or important state changes.
5. **Log Structure**: Use clear, hierarchical paths (e.g., `"Drivetrain/Velocity"`) for easy filtering and analysis.

## FRC Example
```java
@AutoLogOutput
private double elevatorHeight;
Logger.recordOutput("Elevator/Height", elevatorHeight);
Logger.recordOutput("Elevator/AtTarget", isAtTarget());

// Registering a subsystem for bulk logging
AutoLogOutputManager.addObject(elevator);
```

## Best Practices
- **Log Early, Log Often**: Start logging from the beginning of your project.
- **Be Consistent**: Use consistent naming conventions for log paths.
- **Log Key Events**: Log when commands start/finish, when errors occur, and when important state changes happen.
- **Avoid Over-Logging**: Too much data can make it hard to find what matters. Focus on key signals and events.
- **Use AdvantageScope**: Visualize logs to spot trends, glitches, or unexpected behavior.
- **Log in Simulation**: Logging is just as valuable in sim as on real hardware.

## Anti-Patterns to Avoid
- Logging only after problems occur (be proactive)
- Using unclear or inconsistent log paths
- Logging sensitive or unnecessary data
- Relying solely on print statements (use structured logging instead)

## Migration Tips
- Refactor print statements to use `Logger.recordOutput`.
- Annotate important fields with `@AutoLogOutput`.
- Register subsystems or state objects with `AutoLogOutputManager`.
- Review logs after every test and match to build a habit of data-driven debugging.

## Advanced Tips
- **Conditional Logging**: Log only when certain conditions are met to reduce log size.
- **Custom Loggers**: Implement your own logging wrappers for special needs (e.g., error rate tracking).
- **Log Replay**: Use log replay features in AdvantageKit to simulate and debug past matches.
- **Integration with Dashboards**: Use logs to drive dashboard widgets and indicators.

## FRC-Specific Example: Logging Command Lifecycle
```java
public class IntakeCommand extends CommandBase {
    @Override
    public void initialize() {
        Logger.recordOutput("IntakeCommand/Started", true);
    }
    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("IntakeCommand/Ended", interrupted);
    }
}
```

## References
- [AdvantageKit Logging](https://github.com/Mechanical-Advantage/AdvantageKit)
- [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope)
- [WPILib Logging](https://docs.wpilib.org/en/stable/docs/software/telemetry/index.html)
- [FRC Discord Logging Guide](https://discord.com/channels/146771816411586560/1100172954948679780)
