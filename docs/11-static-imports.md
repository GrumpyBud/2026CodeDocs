# Static Imports

## What are Static Imports?
Static imports allow you to use static members (fields and methods) of a class without qualifying them with the class name. This eliminates the need to repeat the class name when referencing static constants or methods, making code more concise and readable, especially for utility classes and constants.

### Basic Syntax
```java
// Regular import of a class
import edu.wpi.first.math.util.Units;

// Static import of specific methods/fields
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.math.util.Units.degreesToRadians;

// Static import of all static members
import static edu.wpi.first.math.util.Units.*;
```

## Why Use Static Imports in FRC?
- **Improved Readability**: Significantly reduces verbosity when using mathematical constants, unit conversions, or utility methods.
- **Cleaner Mathematical Code**: Makes formulas and calculations much easier to read without class prefixes.
- **Clearer Intent**: Focuses on the operation rather than the origin class.
- **Consistent Style**: Creates uniform styling for mathematical and utility operations.
- **Reduces Visual Clutter**: Especially for frequently used utility methods or constants.

## Example from RobotCode2025Public
```java
// Before static imports
public class WithoutStaticImports {
    public void configureTrajectory() {
        double distanceMeters = Units.inchesToMeters(36.0);
        double angleRadians = Units.degreesToRadians(90.0);
        double velocityMetersPerSec = Units.feetToMeters(3.0);
    }
}

// With static imports
import static edu.wpi.first.math.util.Units.*;
import static org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants.*;

public class WithStaticImports {
    public void configureTrajectory() {
        double distanceMeters = inchesToMeters(36.0);
        double angleRadians = degreesToRadians(90.0);
        double velocityMetersPerSec = feetToMeters(3.0);
    }
}
```

## How to Use in FRC
Static imports are particularly useful for:

### 1. Unit Conversions
```java
import static edu.wpi.first.math.util.Units.*;

// Instead of:
double heightMeters = Units.inchesToMeters(48.0);

// You can write:
double heightMeters = inchesToMeters(48.0);
```

### 2. Mathematical Constants and Functions
```java
import static java.lang.Math.*;

// Instead of:
double area = Math.PI * Math.pow(radius, 2);

// You can write:
double area = PI * pow(radius, 2);
```

### 3. FieldConstants and Game-Specific Constants
```java
import static frc.robot.constants.FieldConstants.*;

// Instead of:
Translation2d targetPosition = new Translation2d(FieldConstants.SPEAKER_X, FieldConstants.SPEAKER_Y);

// You can write:
Translation2d targetPosition = new Translation2d(SPEAKER_X, SPEAKER_Y);
```

### 4. Command Factory Methods
```java
import static edu.wpi.first.wpilibj2.command.Commands.*;

// Instead of:
Command sequence = Commands.sequence(
    Commands.runOnce(() -> arm.setPosition(ArmPosition.HIGH)),
    Commands.waitSeconds(1.0),
    Commands.runOnce(() -> intake.eject())
);

// You can write:
Command sequence = sequence(
    runOnce(() -> arm.setPosition(ArmPosition.HIGH)),
    waitSeconds(1.0),
    runOnce(() -> intake.eject())
);
```

## Best Practices

### When to Use Static Imports
- Unit conversion constants and methods (e.g., `Units.inchesToMeters`)
- Mathematical constants and functions (e.g., `Math.PI`, `Math.sin`)
- Command factory methods (e.g., `Commands.sequence`, `Commands.parallel`)
- Game-specific constants defined in a constants class
- Test assertion methods (e.g., `Assertions.assertEquals`)

### When NOT to Use Static Imports
- When it creates ambiguity (multiple static methods with the same name)
- For methods/constants only used once or twice in a file
- For methods where the class name provides important context
- When it might confuse new team members

## Anti-Patterns to Avoid

1. **Importing Everything from Too Many Classes**
```java
// DON'T DO THIS - creates potential name collisions and confusion
import static java.lang.Math.*;
import static java.util.Arrays.*;
import static java.util.Collections.*;
import static edu.wpi.first.math.MathUtil.*;
```

2. **Static Importing Standard Methods**
```java
// DON'T DO THIS - obscures where methods come from
import static java.util.List.of;
import static java.util.Map.entry;
```

3. **Static Importing Non-Constants/Utilities**
```java
// DON'T DO THIS - static importing business logic methods
import static frc.robot.Shooter.calculateOptimalAngle;
```

## Practical FRC Examples

### Trajectory and Path Following
```java
import static edu.wpi.first.math.util.Units.*;
import static java.lang.Math.*;

public class PathGenerator {
    public Trajectory generateScoringPath() {
        // Much cleaner with static imports
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(inchesToMeters(24), inchesToMeters(12)),
                new Translation2d(inchesToMeters(48), inchesToMeters(-12))
            ),
            new Pose2d(inchesToMeters(72), 0, new Rotation2d(degreesToRadians(180))),
            new TrajectoryConfig(feetToMeters(10), feetToMeters(5))
        );
    }
}
```

### Command Composition
```java
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutoFactory {
    public Command getScoreAndMoveCommand() {
        return sequence(
            parallel(
                run(() -> arm.setPosition(HIGH_POSITION)),
                run(() -> drivetrain.stop())
            ),
            waitUntil(arm::isAtTargetPosition),
            runOnce(shooter::shoot),
            waitSeconds(1.0),
            parallel(
                run(() -> arm.setPosition(STOWED_POSITION)),
                runEnd(
                    () -> drivetrain.arcadeDrive(0.5, 0),
                    () -> drivetrain.arcadeDrive(0, 0)
                ).withTimeout(3.0)
            )
        );
    }
}
```

### PID and Control Systems
```java
import static edu.wpi.first.math.controller.PIDController.*;
import static edu.wpi.first.math.util.Units.*;

public class ArmSubsystem extends SubsystemBase {
    private final PIDController controller = new PIDController(
        kP,        // Static import from constants class
        kI,
        kD
    );
    
    public void setPositionDegrees(double degrees) {
        double radians = degreesToRadians(degrees);
        controller.setSetpoint(radians);
    }
}
```

## Migration Tips for FRC Teams

1. **Start with Unit Conversions**: Replace all `Units.X` calls with static imports
2. **Move to Math Functions**: Replace common `Math.X` calls
3. **Create Constants Classes**: Group related constants in well-organized classes
4. **Static Import Command Factories**: For teams using command-based programming
5. **Add Static Imports to Code Guidelines**: Establish team conventions for static imports

## References
- [Java Static Imports](https://docs.oracle.com/javase/8/docs/technotes/guides/language/static-import.html)
- [Google Java Style Guide on Static Imports](https://google.github.io/styleguide/javaguide.html#s3.3-import-statements)
- [WPILib Units Class](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/util/Units.html)
- [Effective Java: Static Import](https://www.oreilly.com/library/view/effective-java-3rd/9780134686097/)
