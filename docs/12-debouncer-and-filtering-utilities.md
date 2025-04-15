# Debouncer and Filtering Utilities

## What are Debouncers and Filters?
Debouncers and filters are utilities that process input signals to produce cleaner, more reliable outputs by eliminating noise, glitches, and unwanted signal characteristics. In FRC, these tools are essential for dealing with real-world sensor inputs that often contain noise, bouncing, or rapid fluctuations.

### Types of Signal Processing Utilities
- **Debouncers**: Prevent rapid oscillation of boolean signals (e.g., button presses, limit switches)
- **Linear Filters**: Process analog signals through mathematical operations (e.g., moving average)
- **Median Filters**: Remove outliers by selecting the median of several samples
- **Slew Rate Limiters**: Limit how quickly a value can change over time
- **Rate Limiters**: Limit the maximum rate of change of a signal

## Why Use Filtering in FRC?
- **Prevent False Triggers**: Avoid false positives from noisy limit switches or beam breaks
- **Smooth Control Inputs**: Create more precise and comfortable driver control
- **Reduce Mechanical Stress**: Prevent rapid oscillations that can damage mechanisms
- **Improve Sensor Reliability**: Make sensing systems more robust in competition
- **Better State Transitions**: More reliable detection of state changes like game piece acquisition
- **Cleaner Data for Logging**: Better data for post-match analysis and debugging

## Example from RobotCode2025Public
```java
// Debouncer for detecting when coral is indexed
private final Debouncer coralIndexedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

// In periodic method:
boolean coralIndexedRaw = coralSensorInputs.data.valid() && 
    coralSensorInputs.data.distanceMeters() <= coralProximity.get();
coralIndexed = coralIndexedDebouncer.calculate(coralIndexedRaw);
```

This example shows how a beam break or proximity sensor reading is debounced to avoid false readings when detecting a game element passing through the system.

## WPILib Filtering Utilities

### 1. Debouncer
Used to filter boolean signals to eliminate rapid oscillations.

```java
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

// Types of debouncers:
// - kRising: Signal must be true for specified time before output becomes true (default)
// - kFalling: Signal must be false for specified time before output becomes false
// - kBoth: Both rising and falling edges are debounced

// Rising edge debouncer (0.1 second)
Debouncer buttonDebouncer = new Debouncer(0.1, DebounceType.kRising);

// Debounce a button input
boolean buttonPressed = joystick.getRawButton(1);
boolean debouncedButton = buttonDebouncer.calculate(buttonPressed);
```

### 2. MedianFilter
Removes outliers by taking the median of a rolling window of samples.

```java
import edu.wpi.first.math.filter.MedianFilter;

// Create a median filter with a window size of 5 samples
MedianFilter filter = new MedianFilter(5);

// Filter a sensor reading
double rawDistance = ultrasonicSensor.getDistance();
double filteredDistance = filter.calculate(rawDistance);
```

### 3. LinearFilter
General-purpose filter that can implement various filtering algorithms (moving average, low-pass, high-pass, etc.)

```java
import edu.wpi.first.math.filter.LinearFilter;

// Create a moving average filter with 10 samples
LinearFilter movingAvg = LinearFilter.movingAverage(10);

// Create a single-pole low-pass filter with time constant 0.1s (for 50Hz updates)
// Time constant relates to how quickly the filter responds to changes
LinearFilter lowPass = LinearFilter.singlePoleIIR(0.1, 0.02); // 0.02s is period (1/50Hz)

// Filter a noisy analog value
double filteredValue = movingAvg.calculate(analogInput.getVoltage());
```

### 4. SlewRateLimiter
Constrains the rate of change of a signal (useful for smoother acceleration/deceleration).

```java
import edu.wpi.first.math.filter.SlewRateLimiter;

// Create a limiter that allows change of at most 2.0 units per second
SlewRateLimiter speedLimiter = new SlewRateLimiter(2.0);

// Limit driver input to prevent jerky movement
double rawThrottle = driverController.getLeftY();
double smoothThrottle = speedLimiter.calculate(rawThrottle);
drivetrain.drive(smoothThrottle, turn);
```

## FRC-Specific Applications

### 1. Reliable Game Piece Detection
```java
public class Intake extends SubsystemBase {
    private final DigitalInput beamBreak = new DigitalInput(0);
    private final Debouncer gamePieceDebouncer = new Debouncer(0.1, DebounceType.kBoth);
    private boolean hasGamePiece = false;
    
    @Override
    public void periodic() {
        // Debounce the beam break to prevent false detections
        boolean beamBroken = !beamBreak.get(); // Inverted because beam breaks return false when broken
        hasGamePiece = gamePieceDebouncer.calculate(beamBroken);
        
        // Log the state
        Logger.recordOutput("Intake/HasGamePiece", hasGamePiece);
    }
    
    public boolean hasGamePiece() {
        return hasGamePiece;
    }
}
```

### 2. Smooth Driver Controls
```java
public class DriveSubsystem extends SubsystemBase {
    private final SlewRateLimiter throttleLimiter = new SlewRateLimiter(3.0); // 3.0 units/sec
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(4.0); // 4.0 units/sec
    
    public void driveWithJoystick(XboxController controller) {
        // Get the raw inputs
        double rawThrottle = -controller.getLeftY(); // Negate because forward is negative on joysticks
        double rawTurn = controller.getRightX();
        
        // Apply deadband
        double throttle = MathUtil.applyDeadband(rawThrottle, 0.1);
        double turn = MathUtil.applyDeadband(rawTurn, 0.1);
        
        // Apply rate limiting for smooth acceleration/turning
        double smoothThrottle = throttleLimiter.calculate(throttle);
        double smoothTurn = turnLimiter.calculate(turn);
        
        // Square inputs for better control while maintaining sign
        double squaredThrottle = Math.copySign(throttle * throttle, throttle);
        double squaredTurn = Math.copySign(turn * turn, turn);
        
        // Drive the robot
        differentialDrive.arcadeDrive(squaredThrottle, squaredTurn);
    }
}
```

### 3. Reliable Limit Switch Detection
```java
public class Elevator extends SubsystemBase {
    private final DigitalInput lowerLimit = new DigitalInput(0);
    private final DigitalInput upperLimit = new DigitalInput(1);
    private final Debouncer lowerLimitDebouncer = new Debouncer(0.05);
    private final Debouncer upperLimitDebouncer = new Debouncer(0.05);
    
    @Override
    public void periodic() {
        boolean rawLowerLimit = !lowerLimit.get(); // Inverted if active low
        boolean rawUpperLimit = !upperLimit.get();
        
        boolean debouncedLowerLimit = lowerLimitDebouncer.calculate(rawLowerLimit);
        boolean debouncedUpperLimit = upperLimitDebouncer.calculate(rawUpperLimit);
        
        // Use debounced values for safety checks
        if (debouncedLowerLimit) {
            motor.stopMotor();
            encoder.setPosition(0.0); // Reset position when at known limit
        }
        
        if (debouncedUpperLimit) {
            motor.stopMotor();
        }
    }
}
```

### 4. Filtering Noisy Sensors
```java
public class DistanceSensor extends SubsystemBase {
    private final AnalogInput ultrasonic = new AnalogInput(0);
    private final MedianFilter medianFilter = new MedianFilter(5);
    private final LinearFilter movingAvgFilter = LinearFilter.movingAverage(10);
    
    private double getVoltageToDistanceMeters(double voltage) {
        // MB1013 ultrasonic sensor: 4.88mV/cm scaling factor
        return voltage / 0.00488;
    }
    
    public double getRawDistanceMeters() {
        return getVoltageToDistanceMeters(ultrasonic.getVoltage());
    }
    
    public double getFilteredDistanceMeters() {
        // First remove outliers with median filter, then smooth with moving average
        double medianFiltered = medianFilter.calculate(ultrasonic.getVoltage());
        double smoothed = movingAvgFilter.calculate(medianFiltered);
        return getVoltageToDistanceMeters(smoothed);
    }
    
    @Override
    public void periodic() {
        Logger.recordOutput("Distance/Raw", getRawDistanceMeters());
        Logger.recordOutput("Distance/Filtered", getFilteredDistanceMeters());
    }
}
```

### 5. Custom Double-Debouncer for Critical Operations
Sometimes you need extra assurance for critical operations like scoring or safety systems:

```java
public class DoubleDebouncedDigitalInput {
    private final DigitalInput input;
    private final Debouncer risingDebouncer;
    private final Debouncer fallingDebouncer;
    private boolean state = false;
    
    public DoubleDebouncedDigitalInput(int channel, double risingTime, double fallingTime) {
        input = new DigitalInput(channel);
        risingDebouncer = new Debouncer(risingTime, DebounceType.kRising);
        fallingDebouncer = new Debouncer(fallingTime, DebounceType.kFalling);
    }
    
    public boolean get() {
        boolean rawValue = input.get();
        boolean risingResult = risingDebouncer.calculate(rawValue);
        boolean fallingResult = fallingDebouncer.calculate(rawValue);
        
        // Only change state if both debouncers agree
        if (risingResult && fallingResult) {
            state = true;
        } else if (!risingResult && !fallingResult) {
            state = false;
        }
        
        return state;
    }
}
```

## Advanced Techniques

### 1. Combining Multiple Filters
For particularly noisy sensors, you might want to combine multiple filtering techniques:

```java
public double getFilteredGyroAngle() {
    // First remove outliers
    double medianFiltered = medianFilter.calculate(gyro.getAngle());
    
    // Then smooth the result
    double smoothed = lowPassFilter.calculate(medianFiltered);
    
    // Finally limit the rate of change for controls
    return rateLimiter.calculate(smoothed);
}
```

### 2. Adaptive Filtering
Adjust filter parameters based on operating conditions:

```java
public void setDriveFiltering(boolean isHighSpeed) {
    if (isHighSpeed) {
        // Less filtering at high speed for responsiveness
        throttleFilter = LinearFilter.singlePoleIIR(0.05, 0.02);
    } else {
        // More filtering at low speed for precision
        throttleFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
    }
}
```

### 3. DoublePressTracker
A useful utility for tracking double-button presses in operator interfaces:

```java
public class DoublePressTracker {
    private final double timeThreshold;
    private double lastPressTime = 0;
    private int pressCount = 0;
    
    public DoublePressTracker(double timeThresholdSeconds) {
        this.timeThreshold = timeThresholdSeconds;
    }
    
    public boolean calculate(boolean currentPress) {
        double currentTime = Timer.getFPGATimestamp();
        
        if (currentPress && lastCurrentState != currentPress) {
            // Rising edge of a press
            if (currentTime - lastPressTime < timeThreshold) {
                // This is a double press
                pressCount = 0;
                lastPressTime = 0;
                return true;
            } else {
                // This is the first press
                pressCount = 1;
                lastPressTime = currentTime;
            }
        }
        
        return false;
    }
}
```

## Anti-Patterns to Avoid

1. **Excessive Debouncing**: Too much debouncing can make controls feel sluggish
   ```java
   // Too much debouncing
   Debouncer excessiveDebouncer = new Debouncer(0.5); // Half second is too long for most buttons
   ```

2. **Wrong Debounce Type**: Using the wrong debounce type for the application
   ```java
   // Wrong: Using rising edge debouncer for detecting when an object leaves a beam break
   Debouncer wrongDebouncer = new Debouncer(0.1, DebounceType.kRising);
   boolean objectGone = wrongDebouncer.calculate(!beamBreak.get());
   
   // Correct: Should use falling edge debouncer for this
   Debouncer correctDebouncer = new Debouncer(0.1, DebounceType.kFalling);
   boolean objectGone = correctDebouncer.calculate(!beamBreak.get());
   ```

3. **Filtering Everything**: Not all signals need filtering
   ```java
   // Don't filter precise digital encoders unnecessarily
   double filteredPosition = movingAverage.calculate(encoder.getPosition());
   ```

4. **Inconsistent Filter Usage**: Using raw readings in some places and filtered in others
   ```java
   // Inconsistent: Sometimes using filtered, sometimes raw
   if (getFilteredDistance() < 5.0) {
       // Do something...
   } else if (getRawDistance() > 10.0) { // Should use filtered distance here too
       // Do something else...
   }
   ```

## Migration Tips for FRC Teams

1. **Identify Noisy Systems**: Look for subsystems with inconsistent behavior or false triggers
2. **Start with Simple Debouncers**: Add basic debouncing to digital inputs first
3. **Add Slew Rate Limiters**: Apply to driving controls for smoother operation
4. **Document Filter Parameters**: Keep track of time constants and window sizes
5. **Test in Various Conditions**: Verify filter behavior across different robot states and field conditions

## References
- [WPILib Debouncer](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/debouncing.html)
- [WPILib Filters](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/index.html)
- [WPILib LinearFilter](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/filter/LinearFilter.html)
- [WPILib MedianFilter](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/filter/MedianFilter.html)
- [WPILib SlewRateLimiter](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/filter/SlewRateLimiter.html)
