# Interface-based IO Abstractions

## What are IO Abstractions?
Interface-based IO abstractions provide a clean separation between hardware interfaces and subsystem logic. This design pattern uses interfaces to define how a subsystem communicates with hardware, with separate implementations for real hardware, simulation, and testing environments.

### Core Components
1. **IO Interface**: Defines methods for hardware interaction
2. **IO Inputs**: Data structure for sensor readings
3. **IO Implementation Classes**: Real hardware and simulated implementations

## Why Use IO Abstractions in FRC?
- **Hardware Independence**: Subsystem logic works regardless of hardware
- **Simulation Support**: Run and test code without hardware
- **Unit Testing**: Test subsystems in isolation with mock hardware
- **Hardware Upgrades**: Swap motor controllers without changing subsystem logic
- **Maintainability**: Clear separation between logic and hardware details

## Example from RobotCode2025Public

```java
// Interface defining hardware interactions
public interface ModuleIO {
    void updateInputs(ModuleIOInputs inputs);
    void setDriveVoltage(double volts);
    void setTurnVoltage(double volts);
}

// Data structure for module inputs
public class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};
    public double[] driveTempCelsius = new double[] {};
    
    public double turnAbsolutePosition = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};
    public double[] turnTempCelsius = new double[] {};
}

// Real hardware implementation using TalonFX
public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder turnEncoder;
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = driveMotor.getPosition().getValueAsDouble();
        inputs.driveVelocityRadPerSec = driveMotor.getVelocity().getValueAsDouble();
        // ... more hardware-specific code
    }
    
    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }
    
    // ... other implementation methods
}

// Simulation implementation
public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update simulation models
        driveSim.update(0.02);
        turnSim.update(0.02);
        
        // Set input values from simulation
        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        // ... more simulation code
    }
    
    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = volts;
        driveSim.setInputVoltage(volts);
    }
    
    // ... other simulation methods
}
```

## How to Implement IO Abstractions in FRC

### Step 1: Define IO Interfaces and Input Classes

```java
// Generic IO interface for an elevator subsystem
public interface ElevatorIO {
    public class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public boolean upperLimitSwitch = false;
        public boolean lowerLimitSwitch = false;
    }
    
    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}
    
    /** Sets the elevator voltage. */
    public default void setVoltage(double volts) {}
    
    /** Sets the elevator brake mode. */
    public default void setBrakeMode(boolean enable) {}
}
```

### Step 2: Create Real Hardware Implementation

```java
public class ElevatorIOSparkMax implements ElevatorIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkLimitSwitch upperLimit;
    private final SparkLimitSwitch lowerLimit;
    
    public ElevatorIOSparkMax() {
        motor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        upperLimit = motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        lowerLimit = motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        
        // Configure motor
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(40);
        motor.setIdleMode(IdleMode.kBrake);
        
        // Configure encoder
        encoder.setPositionConversionFactor(Constants.ELEVATOR_METERS_PER_ROTATION);
        encoder.setVelocityConversionFactor(Constants.ELEVATOR_METERS_PER_ROTATION / 60.0);
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = encoder.getPosition();
        inputs.velocityMetersPerSec = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = new double[] {motor.getOutputCurrent()};
        inputs.upperLimitSwitch = upperLimit.isPressed();
        inputs.lowerLimitSwitch = lowerLimit.isPressed();
    }
    
    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
    
    @Override
    public void setBrakeMode(boolean enable) {
        motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
```

### Step 3: Create Simulation Implementation

```java
public class ElevatorIOSim implements ElevatorIO {
    private final DCMotorSim motorSim;
    private final ElevatorSim elevatorSim;
    private double appliedVolts = 0.0;
    private boolean brakeMode = true;
    
    public ElevatorIOSim() {
        DCMotor motor = DCMotor.getNEO(1);
        motorSim = new DCMotorSim(motor, Constants.ELEVATOR_GEAR_RATIO, 0.025);
        
        elevatorSim = new ElevatorSim(
            motor,
            Constants.ELEVATOR_GEAR_RATIO,
            Constants.ELEVATOR_CARRIAGE_MASS,
            Constants.ELEVATOR_DRUM_RADIUS,
            0.0, // min height
            Constants.ELEVATOR_MAX_HEIGHT,
            brakeMode,
            0.0, // starting position
            VecBuilder.fill(0.001) // standard deviation
        );
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorSim.update(0.02);
        
        inputs.positionMeters = elevatorSim.getPositionMeters();
        inputs.velocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] {motorSim.getCurrentDrawAmps()};
        inputs.upperLimitSwitch = inputs.positionMeters >= Constants.ELEVATOR_MAX_HEIGHT - 0.01;
        inputs.lowerLimitSwitch = inputs.positionMeters <= 0.01;
    }
    
    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        motorSim.setInputVoltage(appliedVolts);
        elevatorSim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void setBrakeMode(boolean enable) {
        brakeMode = enable;
    }
}
```

### Step 4: Create Subsystem That Uses IO Interface

```java
public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();
    
    private final ProfiledPIDController controller;
    private double targetPosition = 0.0;
    
    public Elevator(ElevatorIO io) {
        this.io = io;
        
        controller = new ProfiledPIDController(
            Constants.ELEVATOR_KP,
            Constants.ELEVATOR_KI,
            Constants.ELEVATOR_KD,
            new TrapezoidProfile.Constraints(
                Constants.ELEVATOR_MAX_VELOCITY,
                Constants.ELEVATOR_MAX_ACCELERATION
            )
        );
        controller.setTolerance(0.02); // 2cm tolerance
        
        // Enable continuous input for position controller
        controller.enableContinuousInput(0.0, Constants.ELEVATOR_MAX_HEIGHT);
    }
    
    @Override
    public void periodic() {
        // Update inputs from hardware/simulation
        io.updateInputs(inputs);
        
        // Log all inputs
        Logger.processInputs("Elevator", inputs);
        
        // Calculate and apply control
        if (targetPosition >= 0) {
            double voltage = controller.calculate(inputs.positionMeters, targetPosition);
            voltage += calculateFeedforward();
            io.setVoltage(voltage);
        }
        
        // Log control values
        Logger.recordOutput("Elevator/TargetPosition", targetPosition);
        Logger.recordOutput("Elevator/ControllerOutput", controller.calculate(inputs.positionMeters));
        Logger.recordOutput("Elevator/AtSetpoint", controller.atSetpoint());
    }
    
    /** Sets the target elevator height. */
    public void setPosition(double heightMeters) {
        targetPosition = MathUtil.clamp(
            heightMeters,
            0.0,
            Constants.ELEVATOR_MAX_HEIGHT
        );
    }
    
    /** Stops the elevator and puts it in brake mode. */
    public void stop() {
        io.setVoltage(0.0);
        io.setBrakeMode(true);
        targetPosition = -1; // Disable automatic control
    }
    
    /** Returns true if the elevator is at the target position. */
    public boolean atTargetPosition() {
        return controller.atSetpoint();
    }
    
    /** Returns the current elevator position in meters. */
    public double getPosition() {
        return inputs.positionMeters;
    }
    
    /** Calculate feedforward for gravity compensation. */
    private double calculateFeedforward() {
        return Constants.ELEVATOR_KG * Math.cos(0); // Vertical elevator, no angle
    }
}
```

### Step 5: Dependency Injection in Robot Container

```java
public class RobotContainer {
    // Subsystems with interface-based IO
    private final Elevator elevator;
    private final Intake intake;
    private final Shooter shooter;
    
    public RobotContainer() {
        // Create subsystems with the appropriate IO implementation based on runtime mode
        switch (Constants.getMode()) {
            case REAL:
                // Real robot, use hardware implementations
                elevator = new Elevator(new ElevatorIOSparkMax());
                intake = new Intake(new IntakeIOTalonSRX());
                shooter = new Shooter(new ShooterIOTalonFX());
                break;
                
            case SIM:
                // Simulation, use sim implementations
                elevator = new Elevator(new ElevatorIOSim());
                intake = new Intake(new IntakeIOSim());
                shooter = new Shooter(new ShooterIOSim());
                break;
                
            case REPLAY:
                // Log replay, use dummy implementations
                elevator = new Elevator(new ElevatorIO() {});
                intake = new Intake(new IntakeIO() {});
                shooter = new Shooter(new ShooterIO() {});
                break;
                
            default:
                // Should never happen
                throw new RuntimeException("Unknown mode: " + Constants.getMode());
        }
        
        // Configure button bindings etc.
        configureButtonBindings();
    }
    
    // ... rest of RobotContainer
}
```

## Advanced IO Abstraction Techniques

### Testing with Mock Implementations

```java
public class ElevatorIOMock implements ElevatorIO {
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();
    private double lastVoltage = 0.0;
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Copy our internal inputs to the passed-in inputs
        inputs.positionMeters = this.inputs.positionMeters;
        inputs.velocityMetersPerSec = this.inputs.velocityMetersPerSec;
        inputs.appliedVolts = lastVoltage;
        inputs.currentAmps = new double[] {Math.abs(lastVoltage) * 2.0}; // Simplified model
        inputs.upperLimitSwitch = inputs.positionMeters >= 2.0;
        inputs.lowerLimitSwitch = inputs.positionMeters <= 0.01;
    }
    
    @Override
    public void setVoltage(double volts) {
        lastVoltage = volts;
        
        // Simple model: position changes based on voltage
        if (Math.abs(volts) > 0.1) {
            double direction = Math.signum(volts);
            inputs.velocityMetersPerSec = direction * 0.5; // 0.5 m/s per 12V
            inputs.positionMeters += inputs.velocityMetersPerSec * 0.02; // 20ms loop time
            
            // Clamp position to valid range
            inputs.positionMeters = MathUtil.clamp(inputs.positionMeters, 0.0, 2.0);
        } else {
            inputs.velocityMetersPerSec = 0.0;
        }
    }
    
    @Override
    public void setBrakeMode(boolean enable) {
        // Not needed for testing
    }
    
    // Test-specific methods
    public void setPosition(double positionMeters) {
        inputs.positionMeters = positionMeters;
    }
    
    public void setVelocity(double velocityMetersPerSec) {
        inputs.velocityMetersPerSec = velocityMetersPerSec;
    }
}

// Using the mock in a unit test
@Test
public void testElevatorReachesSetpoint() {
    // Create a mock IO implementation
    ElevatorIOMock io = new ElevatorIOMock();
    
    // Create the elevator subsystem with the mock
    Elevator elevator = new Elevator(io);
    
    // Set initial position
    io.setPosition(0.0);
    
    // Set target position
    elevator.setPosition(1.5);
    
    // Simulate several cycles
    for (int i = 0; i < 100; i++) {
        elevator.periodic();
    }
    
    // Verify elevator reached setpoint
    assertTrue(elevator.atTargetPosition());
    assertEquals(1.5, elevator.getPosition(), 0.02);
}
```

### Replay Implementation for Log Playback

```java
public class ElevatorIOReplay implements ElevatorIO {
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // In replay mode, inputs are populated from logs by AdvantageKit
        // No additional implementation needed
    }
    
    @Override
    public void setVoltage(double volts) {
        // In replay mode, outputs are ignored
    }
    
    @Override
    public void setBrakeMode(boolean enable) {
        // In replay mode, outputs are ignored
    }
}
```

### Multiple Hardware Implementations

```java
// Implementation for SparkMAX + NEO
public class ElevatorIOSparkMAX implements ElevatorIO { /* ... */ }

// Implementation for TalonFX + Falcon
public class ElevatorIOTalonFX implements ElevatorIO { /* ... */ }

// Implementation for REV Through Bore Encoder + Victor SPX
public class ElevatorIOVictorSPX implements ElevatorIO { /* ... */ }
```

## Anti-Patterns to Avoid

1. **Hardware-Specific Logic in Subsystems**
   ```java
   // BAD: Hardware details leak into subsystem
   public class BadElevator extends SubsystemBase {
       private final TalonFX motor = new TalonFX(Constants.ELEVATOR_MOTOR_ID);
       
       public BadElevator() {
           // Hardware-specific configuration
           motor.config_kP(0, 0.1);
           motor.config_kI(0, 0.0);
           motor.config_kD(0, 1.0);
       }
       
       // Hardware-specific methods
       public void setPosition(double positionMeters) {
           motor.set(TalonFXControlMode.Position, positionMeters / Constants.METERS_PER_ROTATION);
       }
   }
   ```

2. **IO Interface with Too Many Methods**
   ```java
   // BAD: Too many specific methods makes implementation complex
   public interface ComplexIO {
       void setPosition(double position);
       void setVelocity(double velocity);
       void setVoltage(double voltage);
       void setCurrent(double current);
       void setMotionMagic(double position);
       void setTrapezoidalProfile(double position);
       // ... too many specific control modes
   }
   
   // BETTER: Simple interface with core functionality
   public interface SimpleIO {
       void updateInputs(Inputs inputs);
       void setVoltage(double voltage);
   }
   ```

3. **Mixing Hardware Interactions with Business Logic**
   ```java
   // BAD: Mixing hardware management with game logic
   @Override
   public void updateInputs(IntakeInputs inputs) {
       inputs.hasCargo = beamBreakSensor.get();
       
       // Don't do game logic here!
       if (inputs.hasCargo) {
           LEDs.setPattern(LEDPattern.CARGO_DETECTED);
           rumbleController();
       }
   }
   ```

## Migration Tips for FRC Teams

1. **Start Small**: Begin with one simple subsystem like a single-motor intake or shooter.

2. **Define IO Interface Standards**: Create consistent interface and input patterns for your team.

3. **Test in Simulation First**: Verify your abstraction works in simulation before trying on hardware.

4. **Use Logging and Replay**: Design with support for logging and replaying logs during testing.

5. **Incremental Migration**: Convert subsystems one at a time rather than all at once.

## References
- [WPILib Simulation Documentation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html)
- [AdvantageKit Documentation](https://github.com/Mechanical-Advantage/AdvantageKit)
- [Team 6328 (Mechanical Advantage) Template](https://github.com/Mechanical-Advantage/RobotTemplate)
- [Team 3015 (Ranger Robotics) Template](https://github.com/RAR1741/RA22_RobotCode)
