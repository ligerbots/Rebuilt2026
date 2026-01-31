// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.CtreTestAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private static double SPEED_LIMIT = 0.5;
    private double MAX_SPEED = SPEED_LIMIT * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public SlewRateLimiter m_headingLimiter = new SlewRateLimiter(10.0); // Limit how fast heading can change
    private static double ROTATION_DEADBAND = 0.2;
    private double m_headingGoal = 0;


    private static final double JOYSTICK_DEADBAND = 0.05;

    /* Heading-based control for driving with target facing direction */
    private final SwerveRequest.FieldCentricFacingAngle m_driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(5, 0, 0); // TODO tune PID gains for heading controller

    // Set the swerve wheels in an X pattern
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry m_logger = new Telemetry(MAX_SPEED);

    private AutoCommandInterface m_autoCommand;

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    // private final CommandJoystick m_farm = new CommandJoystick(1);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagVision m_aprilTagVision = new AprilTagVision();

    public RobotContainer() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        
        // Enable continuous input on the heading controller to prevent 360° rotations
        // The PID controller will now take the shortest path between angles
        m_driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        
        m_drivetrain = new CommandSwerveDrivetrain(
            m_aprilTagVision,
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight
        );

        configureBindings();
    }

    private void configureBindings() {
        m_drivetrain.setupPathPlanner();

        m_drivetrain.setDefaultCommand(getDriveCommand());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // lock wheels
        m_driverController.a().whileTrue(m_drivetrain.applyRequest(() -> m_brakeRequest));
        // m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        m_driverController.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        m_drivetrain.registerTelemetry(m_logger::telemeterize);
    }

    // public Command getAutonomousCommand() {
    //     // Simple drive forward auton
    //     final var idle = new SwerveRequest.Idle();
    //     return Commands.sequence(
    //         // Reset our field centric heading to match the robot
    //         // facing away from our alliance station wall (0 deg).
    //         drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
    //         // Then slowly drive forward (away from us) for 5 seconds.
    //         drivetrain.applyRequest(() ->
    //             drive.withVelocityX(0.5)
    //                 .withVelocityY(0)
    //                 .withRotationalRate(0)
    //         )
    //         .withTimeout(5.0),
    //         // Finally idle for the rest of auton
    //         drivetrain.applyRequest(() -> idle)
    //     );
    // }

    public CommandSwerveDrivetrain getDriveTrain() {
        return m_drivetrain;
    }

    public Command getAutonomousCommand() {
        if(null==m_autoCommand) {
            m_autoCommand = new CtreTestAuto(m_drivetrain, true);
        }
        return m_autoCommand;
    }

    public Pose2d getInitialPose() {
        return ((AutoCommandInterface) getAutonomousCommand()).getInitialPose();
    }    

    public Command getDriveCommand() {
        // The controls are for field-oriented driving with heading control:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> target heading angle (robot maintains this angle)

        return m_drivetrain.applyRequest(() -> {
            // Calculate target heading from right stick X axis
            // Stick position is mapped to a heading angle
            double headingInput = new Rotation2d(-applyRotationDeadband(m_driverController.getRightY()), -applyRotationDeadband(m_driverController.getRightX())).getRotations();
            m_headingGoal = headingInput == 0 ? m_headingGoal : headingInput;
            // System.out.println("Heading input: " + headingInput + " m_goal: " + m_headingGoal);
            SmartDashboard.putNumber("Drivetrain/headingInput", headingInput);
            SmartDashboard.putNumber("Drivetrain/m_headingGoal", m_headingGoal);

            SmartDashboard.putNumber("Drivetrain/robotAngleThinks", m_drivetrain.getPose().getRotation().getRotations());


            Rotation2d targetHeading = Rotation2d.fromRotations(m_headingLimiter.calculate(m_headingGoal));

            

            return m_driveWithHeading
                .withVelocityX(-conditionAxis(m_driverController.getLeftY()) * MAX_SPEED)
                .withVelocityY(-conditionAxis(m_driverController.getLeftX()) * MAX_SPEED)
                .withTargetDirection(targetHeading)
                .withMaxAbsRotationalRate(MAX_ANGULAR_RATE); // Limit rotation speed
        });
    }

    private double conditionAxis(double value) {
        value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);
        // Square the axis, retaining the sign
        return Math.abs(value) * value;
    }

    private double applyRotationDeadband(double value) {
        return MathUtil.applyDeadband(value, ROTATION_DEADBAND);
    }
}
