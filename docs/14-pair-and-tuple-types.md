# Pair and Tuple Types

## What are Pair and Tuple Types?
Pair and tuple types are simple containers for grouping multiple values together. They allow you to bundle related values without creating a full custom class. Java doesn't have built-in tuple types like Python or Kotlin, but WPILib provides the `Pair` class, and you can create or use tuples from various libraries.

### Basic Concepts
- **Pair**: A container holding exactly two related values (often called a 2-tuple)
- **Tuple**: A generalized container for an ordered set of elements (can be 2-tuple, 3-tuple, etc.)
- **Record**: Java's modern alternative to simple tuples (introduced in Java 16)

## Why Use Pair and Tuple Types in FRC?
- **Return Multiple Values**: Return multiple related values from a method without creating a new class
- **Group Related Data**: Keep related data together (e.g., x/y coordinates, distance/angle pairs)
- **Temporary Data Structures**: Quick data grouping for internal processing
- **Avoid Class Explosion**: Prevent creating numerous small classes for one-off data structures
- **Code Clarity**: More meaningful than using arrays for multi-value returns

## Example from RobotCode2025Public
```java
// Using Pair to return multiple values
Pair<Double, Double> calculateDistanceAndAngle(Pose2d target) {
    double distance = currentPose.getTranslation().getDistance(target.getTranslation());
    double angle = new Rotation2d(
        target.getX() - currentPose.getX(),
        target.getY() - currentPose.getY()
    ).getDegrees();
    
    return Pair.of(distance, angle);
}

// Usage
Pair<Double, Double> result = calculateDistanceAndAngle(targetPose);
double distance = result.getFirst();
double angle = result.getSecond();
```

## How to Use Pair in FRC

### WPILib's Pair Class
```java
// Import
import edu.wpi.first.math.Pair;

// Creating a pair
Pair<Integer, String> idAndName = Pair.of(1466, "Webb Robotics");

// Accessing elements
int id = idAndName.getFirst();
String name = idAndName.getSecond();

// Destructuring with var (Java 10+)
var pair = Pair.of(1.5, 3.0);
var first = pair.getFirst();  // Type inferred as Double
var second = pair.getSecond(); // Type inferred as Double
```

### Creating Custom Tuples (3+ elements)
```java
// Custom triple class example
public class Triple<A, B, C> {
    private final A first;
    private final B second; 
    private final C third;
    
    public Triple(A first, B second, C third) {
        this.first = first;
        this.second = second;
        this.third = third;
    }
    
    public A getFirst() { return first; }
    public B getSecond() { return second; }
    public C getThird() { return third; }
    
    public static <A, B, C> Triple<A, B, C> of(A a, B b, C c) {
        return new Triple<>(a, b, c);
    }
}

// Usage
Triple<Double, Double, Boolean> positionData = 
    Triple.of(1.2, 3.4, true); // x, y, isValid
```

### Using Java Records Instead (Java 16+)
```java
// Define a record
public record ShotParameters(double distance, double angle, double rpm) {}

// Create and use
ShotParameters params = new ShotParameters(3.5, 45.0, 4000.0);
double distance = params.distance();
double angle = params.angle();
double rpm = params.rpm();
```

## FRC-Specific Examples

### Shooter Calculations
```java
// Return both the angle and RPM needed to hit a target
public Pair<Double, Double> calculateShot(double distanceMeters) {
    // Simple example calculation
    double angle = 30.0 + (distanceMeters * 5.0); // Higher angle for longer shots
    double rpm = 2000.0 + (distanceMeters * 500.0); // Faster for longer shots
    
    return Pair.of(angle, rpm);
}

// Usage in teleop
public void shootAtTarget() {
    double distance = limelight.getTargetDistance();
    Pair<Double, Double> shotParams = calculateShot(distance);
    
    hood.setAngle(shotParams.getFirst());
    shooter.setRPM(shotParams.getSecond());
}
```

### Vision Processing
```java
// Return both a detected target and its confidence score
public Pair<AprilTagTarget, Double> getBestTarget() {
    List<AprilTagTarget> targets = camera.getDetectedTargets();
    if (targets.isEmpty()) {
        return null;
    }
    
    AprilTagTarget bestTarget = targets.get(0);
    double highestConfidence = bestTarget.getConfidence();
    
    for (AprilTagTarget target : targets) {
        if (target.getConfidence() > highestConfidence) {
            bestTarget = target;
            highestConfidence = target.getConfidence();
        }
    }
    
    return Pair.of(bestTarget, highestConfidence);
}

// Usage
Pair<AprilTagTarget, Double> targetInfo = vision.getBestTarget();
if (targetInfo != null && targetInfo.getSecond() > 0.7) { // Check confidence threshold
    // Use the high-confidence target
    drivetrain.aimAt(targetInfo.getFirst().getPose());
}
```

### Swerve Module States
```java
// Return both wheel speed and angle for a module
public Pair<Double, Rotation2d> calculateModuleState(
        int moduleIndex, ChassisSpeeds speeds) {
    // ... calculation logic ...
    return Pair.of(wheelSpeed, wheelAngle);
}
```

## Best Practices

1. **Use Records When Possible**: For Java 16+, records are generally a better choice than custom tuples
2. **Keep Tuples Small**: Limit to 2-3 elements; use a proper class for more complex data
3. **Use Descriptive Method Names**: Make sure the method name indicates what values are returned
4. **Consider Type Safety**: Pair/tuple elements can be different types, which provides flexibility but can lead to errors
5. **Document Well**: Clearly document what each element in the pair/tuple represents
6. **Use Generic Type Parameters**: Explicitly specify types when creating pairs for clarity

## Anti-Patterns to Avoid

1. **Overusing Pairs**: Don't use pairs when a proper class would be clearer
   ```java
   // AVOID: Returning a pair when the semantics aren't clear
   Pair<Double, Double> getShotParameters(); // What is first vs. second?
   
   // BETTER: Use a clear class/record
   ShotParameters getShotParameters();
   ```

2. **Deeply Nested Pairs**: Avoid creating complex data structures with pairs of pairs
   ```java
   // AVOID: Nested pairs are confusing
   Pair<Pair<Double, Double>, Pair<Boolean, Integer>> complexData;
   
   // BETTER: Use a proper class
   public class ComplexData {
       private final Translation2d position;
       private final boolean isValid;
       private final int count;
       // ...
   }
   ```

3. **Inconsistent Element Order**: Don't switch the order of elements between similar pairs
   ```java
   // AVOID: Inconsistent ordering in related methods
   Pair<Double, Double> getDistanceAndAngle(); // distance first, angle second
   Pair<Double, Double> getTargetAngleAndDistance(); // angle first, distance second
   ```

## Migration Tips for FRC Teams

1. **Identify Array Returns**: Look for methods returning arrays that could be pairs/records
2. **Create Standard Tuple Types**: If your team uses tuples frequently, create standard tuple classes
3. **Consider Records**: If using Java 16+, gradually replace pairs with records
4. **Document in Method Names**: Use clear method names to indicate the returned values
5. **Use Lombok**: Consider using Lombok's `@Value` for simple immutable classes if not using records

## References
- [WPILib Pair Class](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/Pair.html)
- [Java Records](https://docs.oracle.com/en/java/javase/16/language/records.html)
- [Effective Java: Item 65: Prefer interfaces to reflection](https://www.oreilly.com/library/view/effective-java-3rd/9780134686097/)
- [Commons-Lang Triple](https://commons.apache.org/proper/commons-lang/apidocs/org/apache/commons/lang3/tuple/Triple.html)
