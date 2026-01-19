## General Best Practices

### Comments
- Explain the purpose of complex logic, reasoning behind decisions, and assumptions made
- Keep comments concise and to the point—avoid unnecessary verbosity
- Goal: make code readable for developers with basic familiarity, including junior developers
- Place comments on their own line(s) above the element they describe
- Short inline comments after semicolons are appropriate for noting units, etc.

### Variable Names
- Use descriptive, meaningful names that reflect the variable's purpose
- Avoid single-letter names or confusing abbreviations
- Keep names concise while maintaining clarity
- Use `camelCase` for local variables, `m_variableName` for member variables, `UPPER_SNAKE_CASE` for constants

### Function/Method Names
- Use verbs or verb phrases that clearly indicate what the function does
- Provide javadocs with a high-level overview of purpose, parameters, and return value

### Code Structure
- Design functions to be independent with minimal external dependencies
- Promote reusability and testability through componentized, modular design
- Avoid redundancy without sacrificing readability or functionality
- Choose explicit readable code over clever short code—readability always wins

### Formatting
- Follow standard Java formatting conventions (WPILib standards)
- Use 4 spaces per indent (spaces, not tabs)
- Keep lines under 120 characters
- Consistent indentation, spacing, and line breaks improve navigation

## Code Standards

- **Comments**: Clear explanation of complex logic with proper docstrings. Keep concise but don't assume too much prior knowledge.
- **Javadocs**: 
  - Add Javadocs to custom public/private methods. Keep it as short as possible while still being informative.
  - Do NOT add Javadocs to `@Override` methods (`execute()`, `end()`, `initialize()`, `periodic()`, etc.)
  - If Javadocs are preexisting on overridden methods, they can be left as is.
- **Variable Names**: 
  - Member variables (non-constant): `m_variableName` (e.g., `m_speed`, `m_position`)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `MAX_VELOCITY`, `GEAR_RATIO`)
  - Local variables: `camelCase` for consistency with Java conventions
- **Functions**: Verb-based names with javadoc documentation (except `@Override` methods)
- **Error Handling**: Graceful handling of null values and out-of-range inputs
- **Code Style**: Follows standard Java formatting conventions

## FIRST Code Specific Methods Reference

### Rotation2d (edu.wpi.first.math.geometry.Rotation2d)
Represents a 2D rotation. Provides convenient methods for angle arithmetic without manual degree/radian conversions.

**Creation Methods:**
- `Rotation2d.fromDegrees(double)` - Create from degrees
- `Rotation2d.fromRadians(double)` - Create from radians
- `Rotation2d.fromRotations(double)` - Create from motor rotations (useful with TalonFX)
- `Rotation2d.kZero` - Constant for 0 degrees
- `Rotation2d.k180deg` - Constant for 180 degrees

**Arithmetic Operations:**
- `angle1.plus(angle2)` - Add two angles
- `angle1.minus(angle2)` - Subtract two angles (returns `Rotation2d`)
- `angle.times(double)` - Multiply angle by scalar
- `angle.div(double)` - Divide angle by scalar

**Conversion Methods:**
- `angle.getDegrees()` - Get angle in degrees
- `angle.getRadians()` - Get angle in radians
- `angle.getRotations()` - Get angle in rotations (for motor control)

### MathUtil (edu.wpi.first.math.MathUtil)
Common math utilities for FRC programming.

- `MathUtil.clamp(value, low, high)` - Clamps a value between low and high bounds

### Units (edu.wpi.first.math.util.Units)
Unit conversion utilities.

- `Units.degreesToRadians(double)` - Convert degrees to radians
- `Units.radiansToDegrees(double)` - Convert radians to degrees
- `Units.inchesToMeters(double)` - Convert inches to meters
- `Units.metersToInches(double)` - Convert meters to inches

### SmartDashboard (edu.wpi.first.wpilibj.smartdashboard.SmartDashboard)
Publish and subscribe to NetworkTables values for debugging and tuning. Make sure the string key has a path corresponding to the subsystem it is related to (eg "shooter/targetRPM").

- `SmartDashboard.putNumber(String key, double value)` - Publish a number
- `SmartDashboard.putString(String key, String value)` - Publish a string
- `SmartDashboard.putBoolean(String key, boolean value)` - Publish a boolean
- `SmartDashboard.getNumber(String key, double defaultValue)` - Get a number

### SubsystemBase (edu.wpi.first.wpilibj2.command.SubsystemBase)
Base class for all subsystems.

- `periodic()` - Called every scheduler cycle (~20ms). Use for telemetry updates and continuous logic.

### Command (edu.wpi.first.wpilibj2.command.Command)
Base class for all commands.

- `initialize()` - Called once when command starts
- `execute()` - Called every cycle while command runs
- `end(boolean interrupted)` - Called when command ends
- `isFinished()` - Returns true when command should end
- `addRequirements(Subsystem...)` - Declare subsystem dependencies

### Phoenix 6 TalonFX (com.ctre.phoenix6.hardware.TalonFX)
Motor controller for Falcon 500 and Kraken X60 motors.

**Configuration:**
- `getConfigurator().apply(TalonFXConfiguration)` - Apply motor configuration
- `setNeutralMode(NeutralModeValue.Brake/Coast)` - Set behavior when motor is idle
- `setPosition(double)` - Set encoder position (typically 0 at startup)

**Sensor Feedback:**
- `getPosition().getValueAsDouble()` - Get motor position in rotations
- `getVelocity().getValueAsDouble()` - Get motor velocity in rotations per second (RPS)

**Control Requests:**
- `setControl(VelocityVoltage request)` - Closed-loop velocity control
- `setControl(MotionMagicVoltage request)` - Smooth position control with motion profiling

### Phoenix 6 Control Requests
**VelocityVoltage** - Closed-loop velocity control
- `new VelocityVoltage(targetRPS)` - Create request for target rotations per second
- `.withFeedForward(double)` - Add feedforward voltage

**MotionMagicVoltage** - Smooth position control
- `new MotionMagicVoltage(targetRotations)` - Create request for target position in rotations

### Phoenix 6 Configuration Classes
**TalonFXConfiguration** - Main configuration container
- `.Slot0` - PID slot 0 configuration (kP, kI, kD) - There are 5 slots, you initialize a slot at startup and can switch between them during operation (eg. one for an elevator going up and one going down)
- `.CurrentLimits` - Supply and stator current limits
- `.MotionMagic` - Motion Magic cruise velocity and acceleration

**Slot0Configs** - PID configuration
- `kP`, `kI`, `kD` - PID gains

**MotionMagicConfigs** - Motion profiling settings. It is a trapezoidal profile run on the motor controllers.
- `MotionMagicCruiseVelocity` - Target cruise velocity
- `MotionMagicAcceleration` - Target acceleration

**CurrentLimitsConfigs** - Current limiting
- `.withSupplyCurrentLimit(double)` - Limit supply current (from battery)
- `.withStatorCurrentLimit(double)` - Limit stator current (to motor)

## Design Patterns Used in This Codebase

### State Machines
Use enums to track subsystem states for complex behavior. Example from Shooter:
```java
public enum ShooterState {
  IDLE,
  SPINNING_UP,
  READY_TO_SHOOT
}
private ShooterState m_currentState = ShooterState.IDLE;
```
Update state in `periodic()` based on conditions, and expose via `getCurrentState()` for commands to check readiness.

### Lookup Tables
For distance-based calculations (shooting, etc.), use lookup tables loaded from deploy files:
- Store lookup table CSV files in `src/main/deploy/lookupTables/`
- Use `ShooterLookupTable` class to interpolate values based on distance
- Return null or handle gracefully when distance is out of range

### Composite Subsystems
Parent subsystems can own child subsystems for organization. Example: `Shooter` owns `Hood` and `Flywheel`:
```java
private Hood m_hood;
private Flywheel m_flywheel;
```
The parent coordinates behavior and exposes a unified interface.

## Programming Conventions

### Class Naming
- **Subsystem classes**: Use nouns (Shooter, Drivetrain, Hood)
- **Command classes**: Use verbs or verb phrases (Shoot, TurnAndShoot, ShootHub)
- Don't include "Command" or "Subsystem" in class names—it's redundant

### Scope and Visibility
- Prefer `private` for methods and variables unless actually needed outside the class
- Use local variables when values aren't needed outside a single method
- Starting private and making public when needed is better than over-exposing

### NetworkTables Keys
Use subsystem-based paths for SmartDashboard/NetworkTables:
- `"shooter/RPM"` not `"shooterRPM"`
- `"drivetrain/velocity"` not `"drivetrainVelocity"`
- `"turret/goalAngle"` not `"turretGoalAngle"`

### Constants Placement
- **CAN IDs and robot-wide unique values**: Put in `Constants.java`
- **Subsystem-specific constants**: Put in the subsystem class itself
  - Example: `Flywheel.K_P`, `Hood.MAX_ANGLE`
- Don't repeat the class name in the constant: use `MAX_LENGTH` not `REACHER_MAX_LENGTH`

### Subsystem periodic() Usage
Use `periodic()` for:
- SmartDashboard/telemetry updates
- Continuous control loops (sending motor commands based on goals)
- State machine transitions

# Programming Conventions for FRC Java

# Code Formatting

* Generally stick with WPILib standards, except where noted below.  
* **Use consistent line indenting**. 4 spaces per indent is preferred.  
* Indenting should be done using space characters, not tabs. This is much more portable and does not rely on individual user's settings.  
* Lines should generally be no longer than 120 characters.  
* Most comments should be on their own line(s), **above** the element they are describing. Short comments after the semicolon are appropriate to note things like the units of a number, for example.

# Naming Conventions

* Generally, names should use "camel case": word elements which are connected should start with a capital letter.   
  * e.g. motorSpeed, DriveTrain  
* Class names should begin with a capital letter, e.g. Drivetrain, TurnAndShoot  
* Method and variable names should begin with a lowercase letter.  
* Use "m_" prefix for class member variables. No prefix for local variables. This makes it very quick to tell what type of variable each is.  
* Subsystem class names should be "nouns": Shooter, Drivetrain, etc.  
* Command class names should typically be "verbs" or verb phrases: Drive, Shoot, TurnAndShoot.  
  * In most cases, don't include "Command" or "Subsystem" in these names. Redundant.

# Coding Conventions

* Class member variables should not be used for values which are not needed outside of a single routine. That is, local variables should be used whenever possible, and not stored in member variables.  
* Class methods and variables should typically be private unless actually needed outside of the class. Maintenance and learning the code is much easier if the scope of a method/variable is restricted and easily known. It is better to start with a method as private, and make it public when it actually needs to be.

# Robot Code Specific Conventions

* NetworkTable variables should typically be named in a "subdirectory" which is named after the subsystem. So NT variables for the drivetrain should be under /SmartDashboard/drivetrain/. Examples:   
  * /SmartDashboard/shooter/RPM  
  * /SmartDashboard/vision/target\_info  
* Subsystems should be where "permanent" variables are stored, unless they are clearly not part of a subsystem. So if a variable needs to be remembered between commands, it should be put in the appropriate subsystem, not in the main Robot class, where reasonable.  
* The subsystems' "periodic()" methods should be used for functionality which needs to be called every cycle that is related to the subsystem, as opposed to the Robot periodic methods. For example, updating SmartDashboard variables should be in their subsystem's periodic() method.

# Constants

* In Java, constants should be declared "static final" and generally should be all uppercase with "\_" separators:  eg  CLAW\_MOTOR\_CAN\_ID  
* WPILib convention is to put constants into Constants.java in the top level (same level as Robot.java).  
* However, constants which are specifically related to a Subsystem should be put into that subsystem. Examples:  
  * DriveTrain.WHEELBASE\_METERS  
    * Reacher.MAX\_LENGTH  
  * Note: if the constant is in an appropriate class, you don't need to include the class name as part of the name. So, use "Reacher.MAX\_LENGTH" instead of "Reacher.REACHER\_MAX\_LENGTH".  
* **Exceptions**: Constants which need to have unique values across the whole robot should be in Constants.java. A specific example is CAN IDs.