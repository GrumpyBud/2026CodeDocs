# Design Patterns (Builder, Strategy, Singleton, Command, and More)

## What are Design Patterns?
Design patterns are proven, reusable solutions to common software design problems. They provide templates for structuring code in a way that is modular, maintainable, and extensible. In FRC, using design patterns can help you manage the complexity of robot code, especially as your robot and software grow in sophistication.

## Why Use Design Patterns in FRC?
- **Modularity**: Breaks code into independent, reusable components
- **Testability**: Makes it easier to test subsystems and commands in isolation
- **Maintainability**: Simplifies updates and bug fixes by localizing changes
- **Flexibility**: Allows for easy swapping of algorithms, hardware, or behaviors
- **Scalability**: Supports larger codebases and more complex robots

## Key Design Patterns for FRC Robot Code

### 1. Builder Pattern
The Builder pattern separates object construction from its representation, allowing the same construction process to create different representations.

#### FRC Uses:
- Creating complex configuration objects step-by-step
- Building state containers for subsystems 
- Constructing command sequences with many options

#### Example from RobotCode2025Public:
```java
@Builder(toBuilder = true)
public class SuperstructureStateData {
    private final Pose2d pose;
    private final double height;
    private final GripperGoal gripperGoal;
    // ... more fields ...
}

// Usage:
SuperstructureStateData state = SuperstructureStateData.builder()
    .pose(new Pose2d(1, 2, Rotation2d.fromDegrees(90)))
    .height(1.5)
    .gripperGoal(GripperGoal.GRIP)
    .build();

// Modify existing object:
SuperstructureStateData newState = state.toBuilder()
    .height(2.0)
    .build();
```

#### Implementation Options:
1. **Manual Builder Implementation**:
```java
public class TrajectoryConfig {
    private final double maxVelocity;
    private final double maxAcceleration;
    private boolean reversed = false;
    private List<Translation2d> interiorWaypoints = new ArrayList<>();
    
    private TrajectoryConfig(Builder builder) {
        this.maxVelocity = builder.maxVelocity;
        this.maxAcceleration = builder.maxAcceleration;
        this.reversed = builder.reversed;
        this.interiorWaypoints = builder.interiorWaypoints;
    }
    
    public static class Builder {
        private final double maxVelocity;
        private final double maxAcceleration;
        private boolean reversed = false;
        private List<Translation2d> interiorWaypoints = new ArrayList<>();
        
        public Builder(double maxVelocity, double maxAcceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }
        
        public Builder setReversed(boolean reversed) {
            this.reversed = reversed;
            return this;
        }
        
        public Builder addInteriorWaypoint(Translation2d waypoint) {
            this.interiorWaypoints.add(waypoint);
            return this;
        }
        
        public TrajectoryConfig build() {
            return new TrajectoryConfig(this);
        }
    }
}

// Usage:
TrajectoryConfig config = new TrajectoryConfig.Builder(4.0, 3.0)
    .setReversed(true)
    .addInteriorWaypoint(new Translation2d(2, 3))
    .build();
```

2. **Lombok Builder Annotation (Recommended)**:
```java
@Builder
public class ShooterConfig {
    private final double targetRPM;
    private final double hoodAngle;
    @Builder.Default private final boolean backspinEnabled = false;
    @Builder.Default private final double backspinRatio = 1.0;
}

// Usage:
ShooterConfig config = ShooterConfig.builder()
    .targetRPM(4500)
    .hoodAngle(45)
    .backspinEnabled(true)
    .build();
```

### 2. Strategy Pattern
The Strategy pattern defines a family of algorithms, encapsulates each one, and makes them interchangeable. It lets the algorithm vary independently from clients that use it.

#### FRC Uses:
- Different control algorithms (field-oriented vs. robot-oriented drive)
- Autonomous routines for different scenarios
- Shooting or intake strategies for different game pieces

#### Example:
```java
// Interface for drive strategies
public interface DriveStrategy {
    ChassisSpeeds calculateSpeeds(double xInput, double yInput, double rotationInput);
}

// Field-oriented implementation
public class FieldOrientedDrive implements DriveStrategy {
    private final SwerveDrivetrain drivetrain;
    
    @Override
    public ChassisSpeeds calculateSpeeds(double xInput, double yInput, double rotationInput) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xInput * Constants.MAX_VELOCITY,
            yInput * Constants.MAX_VELOCITY,
            rotationInput * Constants.MAX_ANGULAR_VELOCITY,
            drivetrain.getGyroscopeRotation()
        );
    }
}

// Robot-oriented implementation
public class RobotOrientedDrive implements DriveStrategy {
    @Override
    public ChassisSpeeds calculateSpeeds(double xInput, double yInput, double rotationInput) {
        return new ChassisSpeeds(
            xInput * Constants.MAX_VELOCITY,
            yInput * Constants.MAX_VELOCITY,
            rotationInput * Constants.MAX_ANGULAR_VELOCITY
        );
    }
}

// Usage in the drivetrain subsystem
public class Drivetrain extends SubsystemBase {
    private DriveStrategy driveStrategy = new FieldOrientedDrive(this);
    
    public void setDriveStrategy(DriveStrategy strategy) {
        this.driveStrategy = strategy;
    }
    
    public void drive(double xInput, double yInput, double rotationInput) {
        ChassisSpeeds speeds = driveStrategy.calculateSpeeds(xInput, yInput, rotationInput);
        setSpeeds(speeds);
    }
}
```

### 3. Command Pattern
The Command pattern encapsulates a request as an object, allowing for parameterization of clients with different requests, queuing of requests, and logging of the requests.

WPILib's command-based framework is built on this pattern, allowing for composition and scheduling of robot actions.

#### FRC Uses:
- Encapsulating robot actions as commands
- Building complex autonomous sequences
- Binding controls to actions
- Scheduling and cancelling operations

#### Advanced Command Techniques:

**1. Command Composition for Complex Actions**:
```java
public Command scoreHighNode() {
    return Commands.sequence(
        // Drive to scoring position
        new DriveToPositionCommand(scorePosition),
        // Raise the arm
        new SetArmPositionCommand(ArmPosition.HIGH),
        // Wait until arm is in position
        Commands.waitUntil(() -> arm.isAtTarget()),
        // Score the game piece
        new ReleaseGamePieceCommand(),
        // Wait for release
        Commands.waitSeconds(0.5),
        // Return arm to stowed position
        new SetArmPositionCommand(ArmPosition.STOWED)
    );
}
```

**2. Command Factories**:
```java
public class IntakeCommands {
    private final Intake intake;
    
    // Command factory methods
    public Command intakeUntilDetected() {
        return Commands.run(
            () -> intake.setSpeed(Constants.INTAKE_SPEED),
            intake
        ).until(intake::hasGamePiece)
         .andThen(() -> intake.setSpeed(Constants.HOLD_SPEED));
    }
    
    public Command eject() {
        return Commands.runOnce(
            () -> intake.setSpeed(Constants.EJECT_SPEED),
            intake
        ).withTimeout(0.5)
         .andThen(() -> intake.setSpeed(0));
    }
}
```

**3. Command Groups with Deadlines and Parallels**:
```java
// Run multiple commands in parallel with a deadline
public Command intakeWithFeedback() {
    return Commands.deadline(
        // The deadline - stops all commands when this finishes
        intakeCommands.intakeUntilDetected(),
        // Commands to run in parallel with the deadline
        Commands.run(() -> led.setPattern(LED.SEARCHING)),
        Commands.run(() -> rumble.setRumble(0.3))
    );
}

// Run commands in parallel, requiring different subsystems
public Command scoreWithFeedback() {
    return Commands.parallel(
        armCommands.moveToScorePosition(),
        Commands.run(() -> led.setPattern(LED.SCORING))
    );
}
```

**4. Triggers for Event-Driven Commands**:
```java
// Create a trigger that activates when we detect a game piece
new Trigger(intake::hasGamePiece)
    // When the trigger becomes active (game piece detected)
    .onTrue(
        // Sequence of commands to run
        Commands.sequence(
            Commands.runOnce(() -> led.setPattern(LED.GAME_PIECE_DETECTED)),
            Commands.runOnce(() -> haptics.rumbleController(0.5, 0.5)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> led.setPattern(LED.DEFAULT))
        )
    );
```

### 4. Factory Pattern
The Factory pattern provides an interface for creating objects without specifying their concrete classes, allowing for flexible creation of objects.

#### FRC Uses:
- Creating subsystems with different hardware configurations
- Abstract creation of commands or strategies
- Hardware abstraction for simulation and testing

#### Example:
```java
public class SubsystemFactory {
    /**
     * Creates the appropriate intake based on robot type and mode
     */
    public static Intake createIntake() {
        if (Constants.getMode() == Mode.SIM) {
            return new Intake(new IntakeIOSim());
        } else if (Constants.getRobot() == RobotType.COMPETITION) {
            return new Intake(new IntakeIOTalonFX());
        } else {
            return new Intake(new IntakeIOSparkMax());
        }
    }
    
    /**
     * Creates the appropriate drivetrain based on robot type and mode
     */
    public static Drivetrain createDrivetrain() {
        if (Constants.getMode() == Mode.SIM) {
            return new Drivetrain(
                new DrivetrainIOSim(), 
                new GyroIOSim());
        } else if (Constants.getRobot() == RobotType.COMPETITION) {
            return new Drivetrain(
                new DrivetrainIOFalcon(), 
                new GyroIONavX());
        } else {
            return new Drivetrain(
                new DrivetrainIOSparkMax(), 
                new GyroIOPigeon2());
        }
    }
}

// In RobotContainer:
private final Intake intake = SubsystemFactory.createIntake();
private final Drivetrain drivetrain = SubsystemFactory.createDrivetrain();
```

### 5. Singleton Pattern
The Singleton pattern ensures a class has only one instance and provides a global point of access to it.

#### FRC Uses:
- Robot state and data managers
- Logging systems
- Simulation managers

#### Example:
```java
public class RobotState {
    private static RobotState instance;
    
    // Private constructor prevents instantiation from other classes
    private RobotState() {
        // Initialization
    }
    
    // Global access point
    public static synchronized RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
    
    // Robot state data and methods
    private Pose2d robotPose = new Pose2d();
    
    public Pose2d getRobotPose() {
        return robotPose;
    }
    
    public void updatePose(Pose2d newPose) {
        robotPose = newPose;
    }
}

// Usage
RobotState.getInstance().updatePose(new Pose2d(1, 2, new Rotation2d()));
```

#### Anti-Pattern Warning: 
Singletons can make testing difficult and create hidden dependencies. Use sparingly and consider dependency injection instead when possible.

### 6. Observer/Listener Pattern
The Observer pattern defines a one-to-many dependency between objects. When one object changes state, all its dependents are notified and updated automatically.

#### FRC Uses:
- Reacting to sensor readings or state changes
- Updating dashboard displays
- Coordinating subsystem interactions

#### Example:
```java
// Listener interface
public interface GamePieceListener {
    void onGamePieceDetected(GamePieceType type);
    void onGamePieceEjected();
}

// Subsystem with listeners
public class Intake extends SubsystemBase {
    private final List<GamePieceListener> listeners = new ArrayList<>();
    private boolean hasGamePiece = false;
    
    public void addListener(GamePieceListener listener) {
        listeners.add(listener);
    }
    
    @Override
    public void periodic() {
        boolean currentlyHasGamePiece = sensor.get();
        
        // If we just detected a game piece
        if (currentlyHasGamePiece && !hasGamePiece) {
            GamePieceType detectedType = determineGamePieceType();
            // Notify all listeners
            for (GamePieceListener listener : listeners) {
                listener.onGamePieceDetected(detectedType);
            }
        }
        // If we just ejected a game piece
        else if (!currentlyHasGamePiece && hasGamePiece) {
            // Notify all listeners
            for (GamePieceListener listener : listeners) {
                listener.onGamePieceEjected();
            }
        }
        
        hasGamePiece = currentlyHasGamePiece;
    }
}

// Using listeners
public class RobotContainer {
    public RobotContainer() {
        // LED subsystem listens for game piece events
        intake.addListener(new GamePieceListener() {
            @Override
            public void onGamePieceDetected(GamePieceType type) {
                if (type == GamePieceType.CONE) {
                    led.setPattern(LEDPattern.CONE_DETECTED);
                } else {
                    led.setPattern(LEDPattern.CUBE_DETECTED);
                }
            }
            
            @Override
            public void onGamePieceEjected() {
                led.setPattern(LEDPattern.DEFAULT);
            }
        });
    }
}
```

### 7. State Pattern
The State pattern allows an object to alter its behavior when its internal state changes, appearing to change its class.

#### FRC Uses:
- Implementing subsystem state machines
- Robot mode management
- Game piece handling logic

#### Example:
```java
// State interface
public interface ElevatorState {
    void update(Elevator elevator);
    ElevatorState handleLimitSwitch(Elevator elevator, boolean upperLimit, boolean lowerLimit);
}

// Concrete states
public class IdleState implements ElevatorState {
    @Override
    public void update(Elevator elevator) {
        elevator.setMotorOutput(0);
    }
    
    @Override
    public ElevatorState handleLimitSwitch(Elevator elevator, boolean upperLimit, boolean lowerLimit) {
        // No state transition when idle
        return this;
    }
}

public class MovingUpState implements ElevatorState {
    @Override
    public void update(Elevator elevator) {
        elevator.setMotorOutput(0.7); // Move up at 70% power
    }
    
    @Override
    public ElevatorState handleLimitSwitch(Elevator elevator, boolean upperLimit, boolean lowerLimit) {
        if (upperLimit) {
            // Transition to idle if we hit the upper limit
            return new IdleState();
        }
        return this;
    }
}

// Elevator subsystem using states
public class Elevator extends SubsystemBase {
    private ElevatorState currentState = new IdleState();
    private final DigitalInput upperLimitSwitch = new DigitalInput(0);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(1);
    private final WPI_TalonSRX motor = new WPI_TalonSRX(1);
    
    @Override
    public void periodic() {
        // Update based on current state
        currentState.update(this);
        
        // Check for state transitions
        boolean upperLimit = upperLimitSwitch.get();
        boolean lowerLimit = lowerLimitSwitch.get();
        ElevatorState newState = currentState.handleLimitSwitch(this, upperLimit, lowerLimit);
        
        if (newState != currentState) {
            currentState = newState;
        }
    }
    
    public void moveUp() {
        currentState = new MovingUpState();
    }
    
    public void moveDown() {
        currentState = new MovingDownState();
    }
    
    public void stop() {
        currentState = new IdleState();
    }
    
    // Called by states
    public void setMotorOutput(double output) {
        motor.set(output);
    }
}
```

### 8. Decorator Pattern
The Decorator pattern attaches additional responsibilities to an object dynamically, providing a flexible alternative to subclassing.

#### FRC Uses:
- Adding features to commands
- Enhancing sensor inputs with filters
- Adding logging or timing to operations

#### Example:
```java
// Base command
public class DriveDistanceCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final double distance;
    
    public DriveDistanceCommand(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        drivetrain.resetEncoders();
    }
    
    @Override
    public void execute() {
        drivetrain.arcadeDrive(0.5, 0);
    }
    
    @Override
    public boolean isFinished() {
        return drivetrain.getAverageEncoderDistance() >= distance;
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
}

// Decorator to add timeout
public class TimeoutDecorator extends CommandBase {
    private final Command command;
    private final double timeout;
    private double startTime;
    
    public TimeoutDecorator(Command command, double timeout) {
        this.command = command;
        this.timeout = timeout;
        addRequirements(command.getRequirements().toArray(new Subsystem[0]));
    }
    
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        command.initialize();
    }
    
    @Override
    public void execute() {
        command.execute();
    }
    
    @Override
    public boolean isFinished() {
        // Original command is finished OR timeout elapsed
        return command.isFinished() || (Timer.getFPGATimestamp() - startTime >= timeout);
    }
    
    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}

// Usage
Command driveCommand = new DriveDistanceCommand(drivetrain, 2.0); // Drive 2 meters
Command withTimeout = new TimeoutDecorator(driveCommand, 5.0); // With 5 second timeout
```

## Common Design Pattern Combinations in FRC

### State Machine + Command Pattern
Combining state machines with commands creates dynamic behavior trees:

```java
public class SuperstructureStateMachine {
    private final Map<SuperstructureState, Map<Trigger, SuperstructureState>> stateTransitions = new HashMap<>();
    private SuperstructureState currentState = SuperstructureState.STOWED;
    
    public SuperstructureStateMachine() {
        // Define state transitions
        addTransition(SuperstructureState.STOWED, SuperstructureState.INTAKING, 
                      () -> operator.getIntakeButton());
        addTransition(SuperstructureState.INTAKING, SuperstructureState.STOWED, 
                      () -> intake.hasGamePiece() || operator.getStowButton());
        // ... more transitions
    }
    
    private void addTransition(SuperstructureState from, SuperstructureState to, BooleanSupplier trigger) {
        stateTransitions.computeIfAbsent(from, k -> new HashMap<>())
            .put(new Trigger(trigger), to);
    }
    
    public void periodic() {
        // Check all transitions from current state
        stateTransitions.getOrDefault(currentState, Map.of())
            .forEach((trigger, targetState) -> {
                if (trigger.getAsBoolean()) {
                    transitionTo(targetState);
                }
            });
    }
    
    private void transitionTo(SuperstructureState newState) {
        // Run commands associated with state transition
        CommandScheduler.getInstance()
            .schedule(getTransitionCommand(currentState, newState));
        currentState = newState;
    }
    
    private Command getTransitionCommand(SuperstructureState from, SuperstructureState to) {
        return switch (to) {
            case STOWED -> Commands.sequence(
                intake.stopCommand(),
                arm.stowCommand()
            );
            case INTAKING -> Commands.sequence(
                arm.intakePositionCommand(),
                intake.runCommand()
            );
            // ... more states
            default -> Commands.none();
        };
    }
}
```

### Factory + Strategy Pattern
Using factories to create different strategies creates flexible, configurable behavior:

```java
public class AutoModeFactory {
    public static AutoMode createAutoMode(String autoModeName) {
        return switch (autoModeName) {
            case "ScoreAndBalance" -> new ScoreAndBalanceAuto();
            case "ThreePiece" -> new ThreePieceAuto();
            case "MobilityOnly" -> new MobilityAuto();
            case "DoNothing" -> new DoNothingAuto();
            default -> new DoNothingAuto();
        };
    }
}

public interface AutoMode {
    Command getCommand();
}

public class ScoreAndBalanceAuto implements AutoMode {
    @Override
    public Command getCommand() {
        return Commands.sequence(
            new ScoreInitialGamePiece(),
            new DriveToChargeStation(),
            new BalanceChargeStation()
        );
    }
}
```

### Builder + Composite Pattern
Using builders with composite commands creates readable, flexible command sequences:

```java
public class AutoCommandBuilder {
    private final List<Command> commands = new ArrayList<>();
    
    public AutoCommandBuilder score() {
        commands.add(new ScoreGamePieceCommand());
        return this;
    }
    
    public AutoCommandBuilder drive(double distance) {
        commands.add(new DriveDistanceCommand(distance));
        return this;
    }
    
    public AutoCommandBuilder intake() {
        commands.add(
            Commands.deadline(
                Commands.waitUntil(() -> intake.hasGamePiece()),
                Commands.parallel(
                    intake.runCommand(),
                    new DriveToGamePieceCommand()
                )
            )
        );
        return this;
    }
    
    public AutoCommandBuilder balance() {
        commands.add(new BalanceCommand());
        return this;
    }
    
    public Command build() {
        return Commands.sequence(commands.toArray(Command[]::new));
    }
}

// Usage
Command twoGamePieceAuto = new AutoCommandBuilder()
    .score()           // Score preload
    .drive(2)          // Drive 2 meters
    .intake()          // Intake next game piece
    .drive(-2)         // Drive back
    .score()           // Score again
    .balance()         // Balance on charging station
    .build();
```

## Anti-Patterns to Avoid in FRC

### 1. God Subsystem/Command
Avoid creating massive subsystems or commands that do too many things:

```java
// AVOID: Subsystem doing too much
public class MegaSubsystem extends SubsystemBase {
    // Handles driving, shooting, climbing, intake, etc.
    // 1000+ lines of code
}

// BETTER: Separate into logical subsystems
public class Drivetrain extends SubsystemBase { /* ... */ }
public class Shooter extends SubsystemBase { /* ... */ }
public class Climber extends SubsystemBase { /* ... */ }
public class Intake extends SubsystemBase { /* ... */ }
```

### 2. Subsystem Coupling
Avoid direct dependencies between subsystems:

```java
// AVOID: Direct dependencies
public class Intake extends SubsystemBase {
    private final Arm arm; // Direct reference to another subsystem
    
    public Intake(Arm arm) {
        this.arm = arm;
    }
    
    @Override
    public void periodic() {
        // Directly controlling another subsystem
        if (hasGamePiece()) {
            arm.setPosition(ArmPosition.STOWED);
        }
    }
}

// BETTER: Use commands or a coordinator
public class RobotContainer {
    public RobotContainer() {
        // LED subsystem listens for game piece events
        intake.addListener(new GamePieceListener() {
            @Override
            public void onGamePieceDetected(GamePieceType type) {
                if (type == GamePieceType.CONE) {
                    led.setPattern(LEDPattern.CONE_DETECTED);
                } else {
                    led.setPattern(LEDPattern.CUBE_DETECTED);
                }
            }
            
            @Override
            public void onGamePieceEjected() {
                led.setPattern(LEDPattern.DEFAULT);
            }
        });
    }
}
```

### 3. Magic Numbers
Avoid hardcoding values throughout your code:

```java
// AVOID: Magic numbers
public void setShooterSpeed() {
    shooter.setRPM(3500);
    shooter.setHoodAngle(45);
}

public void setIntakeSpeed() {
    intake.setSpeed(0.5);
}

// BETTER: Named constants
public class ShooterConstants {
    public static final double HIGH_GOAL_RPM = 3500;
    public static final double HIGH_GOAL_ANGLE = 45;
}

public class IntakeConstants {
    public static final double INTAKE_SPEED = 0.5;
}

// Usage
public void setShooterForHighGoal() {
    shooter.setRPM(ShooterConstants.HIGH_GOAL_RPM);
    shooter.setHoodAngle(ShooterConstants.HIGH_GOAL_ANGLE);
}

public void setIntake() {
    intake.setSpeed(IntakeConstants.INTAKE_SPEED);
}
```

### 4. Overusing Singletons
Avoid making everything a singleton:

```java
// AVOID: Singleton everything
public class Drivetrain {
    private static Drivetrain instance;
    public static Drivetrain getInstance() { /* ... */ }
}

public class Intake {
    private static Intake instance;
    public static Intake getInstance() { /* ... */ }
}

// BETTER: Dependency injection
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    // Pass as parameters to commands and other classes
}
```

## Migration Tips for FRC Teams

### 1. Start with Command Composition
The command pattern is built into WPILib, making it an easy entry point:

```java
// Instead of monolithic commands:
public class ComplexAutoCommand extends CommandBase {
    // 200+ lines of code managing multiple subsystems
}

// Break down into composable pieces:
public Command getComplexAutoCommand() {
    return Commands.sequence(
        new ScorePreloadCommand(),
        new DriveToPickupCommand(),
        new IntakeBallCommand(),
        new DriveToScoreCommand(),
        new ScoreCommand()
    );
}
```

### 2. Extract Strategies from Duplicated Code
Look for similar code with slight variations and extract to strategies:

```java
// Before: Duplicated shooting logic
public void shootHighGoal() {
    shooter.setRPM(3500);
    shooter.setHoodAngle(45);
}

public void shootLowGoal() {
    shooter.setRPM(2000);
    shooter.setHoodAngle(20);
}

// After: Strategy pattern
public interface ShootingStrategy {
    void configureShooter(Shooter shooter);
}

public class HighGoalStrategy implements ShootingStrategy {
    @Override
    public void configureShooter(Shooter shooter) {
        shooter.setRPM(3500);
        shooter.setHoodAngle(45);
    }
}

public class LowGoalStrategy implements ShootingStrategy {
    @Override
    public void configureShooter(Shooter shooter) {
        shooter.setRPM(2000);
        shooter.setHoodAngle(20);
    }
}

// Usage
public void shoot(ShootingStrategy strategy) {
    strategy.configureShooter(shooter);
    shooter.shoot();
}
```

### 3. Replace Complex Configuration with Builders
Use builders for complex configuration objects:

```java
// Before: Complex constructor
TrajectoryConfig config = new TrajectoryConfig(
    4.0, 3.0, true, false, List.of(new Translation2d(1, 1)), 
    0.3, true, true, 0.5
);

// After: Builder pattern
TrajectoryConfig config = TrajectoryConfig.builder()
    .setMaxVelocity(4.0)
    .setMaxAcceleration(3.0)
    .setReversed(true)
    .addInteriorWaypoint(new Translation2d(1, 1))
    .setEndVelocity(0.3)
    .build();
```

### 4. Use Factory Methods for Complex Objects
Create factory methods for complex objects:

```java
// Before: Complex object creation scattered everywhere
Command autoCommand;
if (autoSelector.equals("TwoBall")) {
    autoCommand = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new IntakeCommand(),
            new DriveToPositionCommand(new Pose2d(1, 2, new Rotation2d()))
        ),
        new WaitCommand(0.5),
        new ShootCommand()
    );
} else if (autoSelector.equals("OneBall")) {
    // Another complex sequence...
}

// After: Factory method
public class AutoCommandFactory {
    public static Command createAutoCommand(String autoName) {
        return switch (autoName) {
            case "TwoBall" -> createTwoBallCommand();
            case "OneBall" -> createOneBallCommand();
            default -> Commands.none();
        };
    }
    
    private static Command createTwoBallCommand() {
        return Commands.sequence(
            Commands.parallel(
                new IntakeCommand(),
                new DriveToPositionCommand(new Pose2d(1, 2, new Rotation2d()))
            ),
            Commands.waitSeconds(0.5),
            new ShootCommand()
        );
    }
    
    private static Command createOneBallCommand() {
        // ...
    }
}

// Usage
Command autoCommand = AutoCommandFactory.createAutoCommand(autoSelector);
```

## FRC-Specific Design Pattern Examples

### 1. Autonomous Command Selection Pattern

```java
public interface AutoRoutine {
    String getName();
    Command getCommand();
}

public class TwoBallAuto implements AutoRoutine {
    @Override
    public String getName() {
        return "Two Ball Auto";
    }
    
    @Override
    public Command getCommand() {
        return Commands.sequence(
            // Commands for two ball auto
            new ScorePreloadCommand(),
            new DriveToPickupCommand(),
            new IntakeBallCommand(),
            new DriveToScoreCommand(),
            new ScoreCommand()
        );
    }
}

public class RobotContainer {
    private final List<AutoRoutine> autoRoutines = List.of(
        new DoNothingAuto(),
        new OneBallAuto(),
        new TwoBallAuto(),
        new ThreeBallAuto(),
        new TaxiOnlyAuto()
    );
    
    private final SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();
    
    public RobotContainer() {
        // Configure auto chooser
        for (AutoRoutine routine : autoRoutines) {
            if (routine == autoRoutines.get(0)) {
                autoChooser.setDefaultOption(routine.getName(), routine);
            } else {
                autoChooser.addOption(routine.getName(), routine);
            }
        }
        SmartDashboard.putData("Auto Routine", autoChooser);
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected().getCommand();
    }
}
```

### 2. Subsystem State Machine Pattern

```java
public enum ArmState {
    STOWED, INTAKING, SCORING_LOW, SCORING_MID, SCORING_HIGH
}

public class Arm extends SubsystemBase {
    private ArmState currentState = ArmState.STOWED;
    private final Map<ArmState, ArmPosition> statePositions = Map.of(
        ArmState.STOWED, new ArmPosition(0.1, 0.1),
        ArmState.INTAKING, new ArmPosition(-0.5, 0.3),
        ArmState.SCORING_LOW, new ArmPosition(0.2, 0.4),
        ArmState