# Java Records

## What are Java Records?
Java Records, introduced in Java 16, are a concise way to create immutable data classes. They automatically generate constructors, accessors, `equals()`, `hashCode()`, and `toString()` methods. Records provide a compact syntax for creating classes that are primarily data carriers.

```java
// Traditional class (70+ lines with all methods fully implemented)
public class Point {
    private final int x;
    private final int y;
    
    public Point(int x, int y) {
        this.x = x;
        this.y = y;
    }
    
    public int getX() { return x; }
    public int getY() { return y; }
    
    @Override public boolean equals(Object o) { /* ... */ }
    @Override public int hashCode() { /* ... */ }
    @Override public String toString() { /* ... */ }
}

// Equivalent record (one line!)
public record Point(int x, int y) {}
```

## Why Use Records in FRC?
- **Reduced Boilerplate**: Dramatically reduce code for data containers
- **Immutability**: Prevent accidental modification of important values
- **Type Safety**: Replace generic tuples or arrays with named, typed fields
- **Clear Intent**: Signal that a class is a pure data carrier
- **Auto-generated Methods**: Get `equals()`, `hashCode()`, and `toString()` for free
- **Pattern Matching**: Work well with Java's pattern matching features

## Examples from RobotCode2025Public
```java
// Simple data record with typed fields
public record CoralObjective(int branchId, ReefLevel reefLevel) {}

// Record with a custom constructor for validation or defaults
public record AlgaeObjective(int id, boolean low) {
    public AlgaeObjective(int id) {
        this(id, false);
    }
}
```

## How Records Improve FRC Code

### 1. Robot State Snapshots
Records are perfect for capturing point-in-time values from sensors or systems:

```java
public record DrivetrainState(
    Pose2d pose,
    ChassisSpeeds velocities,
    double timestamp
) {}

// Usage
DrivetrainState currentState = new DrivetrainState(
    drivetrain.getPose(),
    drivetrain.getChassisSpeeds(),
    Timer.getFPGATimestamp()
);
```

### 2. Game-Specific Elements
Model game-specific elements as records for type safety:

```java
public record ScoringPosition(int gridIndex, int nodeIndex, boolean isHigh) {}

// Usage
ScoringPosition target = new ScoringPosition(2, 1, true);
autoAim(target);
```

### 3. Configuration Parameters
Group related configuration values:

```java
public record PIDConfig(double kP, double kI, double kD) {}

// Usage
PIDConfig shooterPID = new PIDConfig(0.1, 0.0, 0.05);
configureMotor(motor, shooterPID);
```

### 4. API Responses and Events
Pass structured data between subsystems:

```java
public record VisionMeasurement(
    Pose3d targetPose,
    double latencySeconds,
    double confidence
) {}

// Usage
Optional<VisionMeasurement> result = vision.getLatestMeasurement();
result.ifPresent(m -> robotState.addVisionMeasurement(m));
```

## Advanced Record Usage in FRC

### Custom Constructors
Add validation or provide convenient overloads:

```java
public record ShooterParameters(double rpm, double hoodAngle, double backspinRatio) {
    // Validation in canonical constructor
    public ShooterParameters {
        if (rpm < 0 || rpm > 6000) {
            throw new IllegalArgumentException("RPM out of range: " + rpm);
        }
        if (hoodAngle < 0 || hoodAngle > 45) {
            throw new IllegalArgumentException("Hood angle out of range: " + hoodAngle);
        }
    }
    
    // Compact constructor overload
    public ShooterParameters(double rpm, double hoodAngle) {
        this(rpm, hoodAngle, 1.0); // Default backspin ratio
    }
}
```

### Static Factory Methods
Add factory methods for common instances:

```java
public record ArmPosition(double shoulderRadians, double wristRadians) {
    public static ArmPosition stowed() {
        return new ArmPosition(0.1, 0.2);
    }
    
    public static ArmPosition scoring() {
        return new ArmPosition(1.2, 0.8);
    }
    
    public static ArmPosition floor() {
        return new ArmPosition(-0.7, 0.5);
    }
}
```

### Instance Methods
Add methods to perform operations on the record data:

```java
public record Pose2dWithConfidence(Pose2d pose, double confidence) {
    public Pose2dWithConfidence withRotation(Rotation2d newRotation) {
        return new Pose2dWithConfidence(
            new Pose2d(pose.getTranslation(), newRotation),
            confidence
        );
    }
    
    public boolean isReliable() {
        return confidence > 0.7;
    }
}
```

### Nested Records
Compose records for hierarchical data:

```java
public record ModuleState(double speedMetersPerSecond, Rotation2d angle) {}

public record DriveState(
    ModuleState frontLeft,
    ModuleState frontRight,
    ModuleState backLeft,
    ModuleState backRight
) {
    public double getAverageSpeed() {
        return (frontLeft.speedMetersPerSecond() + 
                frontRight.speedMetersPerSecond() +
                backLeft.speedMetersPerSecond() + 
                backRight.speedMetersPerSecond()) / 4.0;
    }
}
```

## When to Use (and Not Use) Records

### Use Records For:
- Data transfer objects
- Value objects and measurements
- Configuration parameters
- API responses
- Immutable data containers
- Return types for multiple values

### Don't Use Records For:
- Mutable state that needs to change after creation
- Classes with complex behavior beyond data access
- Classes requiring inheritance (records cannot extend other classes)
- Implementing frameworks that expect traditional classes

## Anti-Patterns to Avoid

1. **Adding Mutable Fields**: Records are meant to be immutable. Don't try to add mutable fields.
   ```java
   // DON'T DO THIS
   public record BadRecord(int x, int y) {
       private List<Integer> history = new ArrayList<>();
       
       public void addToHistory(int value) {
           history.add(value); // Breaks immutability!
       }
   }
   ```

2. **Too Many Fields**: Records with many fields become hard to use. Consider splitting them.
   ```java
   // Too many fields
   public record OvercomplicatedConfig(
       double kP, double kI, double kD, double kF,
       double maxVel, double maxAccel, double tolerance,
       double timeout, double minOutput, double maxOutput,
       boolean invertMotor, boolean enableSoftLimits,
       double forwardSoftLimit, double reverseSoftLimit
   ) {}
   
   // Better: Split into logical groups
   public record PIDConfig(double kP, double kI, double kD, double kF) {}
   public record MotionConfig(double maxVel, double maxAccel) {}
   public record LimitConfig(double minOutput, double maxOutput) {}
   ```

3. **Using For Subsystems or Commands**: Records aren't suitable for classes with behavior.

## Migration Tips for FRC Teams

1. **Start with Simple Data Classes**: Look for POJOs (Plain Old Java Objects) that mainly hold data.

2. **Replace Tuple Returns**: Change methods returning arrays/tuples to return records.
   ```java
   // Before
   double[] getShooterParams() {
       return new double[] {rpm, angle};
   }
   
   // After
   record ShooterParams(double rpm, double angle) {}
   ShooterParams getShooterParams() {
       return new ShooterParams(rpm, angle);
   }
   ```

3. **Bundle Configuration**: Group related configuration parameters.

4. **Create Sensor Reading Records**: Snapshot sensor readings in records.

5. **Standardize Game Element Representation**: Use records for game pieces, scoring positions, etc.

## FRC-Specific Example: Tracking Field Elements

```java
// Game piece tracking
public record GamePieceLocation(
    Translation2d position,
    GamePieceType type,
    double timestamp,
    double confidenceScore
) {
    public boolean isStale() {
        return Timer.getFPGATimestamp() - timestamp > 0.5;
    }
    
    public GamePieceLocation withUpdatedPosition(Translation2d newPosition) {
        return new GamePieceLocation(
            newPosition,
            this.type,
            Timer.getFPGATimestamp(),
            this.confidenceScore
        );
    }
}

// Usage
List<GamePieceLocation> visibleGamePieces = vision.detectGamePieces();
for (GamePieceLocation piece : visibleGamePieces) {
    if (!piece.isStale() && piece.confidenceScore() > 0.8) {
        targetPiece = piece;
        break;
    }
}
```

## References
- [Java Records (JEP 395)](https://openjdk.java.net/jeps/395)
- [Records in Java](https://docs.oracle.com/en/java/javase/16/language/records.html)
- [Effective Java, 3rd Edition (Joshua Bloch)](https://www.oreilly.com/library/view/effective-java-3rd/9780134686097/)
