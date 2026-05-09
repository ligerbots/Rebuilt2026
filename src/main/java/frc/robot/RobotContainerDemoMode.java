// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.*;
import frc.robot.commands.autoCommands.AutoCommandInterface;
import frc.robot.generated.TunerConstantsCompBot;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.DataLogger;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShotType;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.Turret;

public class RobotContainerDemoMode extends RobotContainer {
    private static final double SPEED_LIMIT = 1.0;
    private double MAX_SPEED = SPEED_LIMIT * TunerConstantsCompBot.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // private static final double JOYSTICK_DEADBAND = 0.05;

    // /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // set the swerve wheels in an X pattern
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry m_logger = new Telemetry(MAX_SPEED);

    private AutoCommandInterface m_autoCommand;

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagVision m_aprilTagVision = new AprilTagVision(Robot.RobotType.COMPBOT, m_logger.getField2d());
    private final ShooterFeeder m_shooterFeeder = new ShooterFeeder();
    private final Shooter m_shooter = new Shooter();
    private final Turret m_turret = new Turret(m_logger.getField2d());

    private final Intake m_intake = new Intake();
    private final Hopper m_hopper;

    // not used directly, but the periodic() method logs data
    @SuppressWarnings("unused")
    private final DataLogger m_dataLogger = new DataLogger();

    public RobotContainerDemoMode() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        
        m_drivetrain = new CommandSwerveDrivetrain(
            m_aprilTagVision,
            TunerConstantsCompBot.DrivetrainConstants,
            TunerConstantsCompBot.FrontLeft, TunerConstantsCompBot.FrontRight, TunerConstantsCompBot.BackLeft, TunerConstantsCompBot.BackRight
        );
        m_hopper = new Hopper(m_drivetrain::getRobotCentricSpeeds); 

        m_drivetrain.setupPathPlanner();

        configureBindings();
    }

    public Command getShootCommand() {
        return withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, ShotType.AUTO));
                        //     new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true)));
    }
    
    private void configureBindings() {
        // No Driving in Demo mode; lock wheels
        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(() -> m_brakeRequest));

        // enable/disable brake mode on the pivot when the robot is disabled
        RobotModeTriggers.disabled().onFalse(new InstantCommand(() -> m_intake.getPivot().setBrakeMode(false)));
        RobotModeTriggers.disabled().onTrue(
            new InstantCommand(() -> m_intake.getPivot().setBrakeMode(true)).ignoringDisable(true)
        );
                             
        // Deploy and run the intake (intake will stay out)
        m_driverController.leftTrigger().onTrue(m_intake.getPivot().deployCommand());
        m_driverController.leftTrigger().whileTrue(
                new StartEndCommand(m_intake.getRoller()::fastIntake, m_intake.getRoller()::stop, m_intake.getRoller()));
                        // .alongWith(new StartEndCommand(m_hopper::intake, m_hopper::stop, m_hopper)));

        // Stow the intake
        m_driverController.leftBumper().onTrue(m_intake.stowCommand());

        // lock wheels
        m_driverController.back().whileTrue(m_drivetrain.applyRequest(() -> m_brakeRequest));

        // Demo shots: fixed arc, roughly the normal Hub shot, set turret to +/- 45deg and forward
        // Forward
        m_driverController.y().whileTrue(withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder,
                        m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, 100.0, Rotation2d.kZero)));
        // Left
        m_driverController.x().whileTrue(withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder,
                        m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, 100.0, Rotation2d.fromDegrees(45))));
        // Right
        m_driverController.b().whileTrue(withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder,
                        m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, 100.0, Rotation2d.fromDegrees(-45))));

        // set the intake sensor position assuming it is deployed
        m_driverController.back().onTrue(new InstantCommand(() -> m_intake.getPivot().setPositionToDeployed()));
        m_driverController.start().onTrue(new InstantCommand(() -> m_intake.getPivot().setPositionToDeployed()));

        m_drivetrain.registerTelemetry(m_logger::telemeterize);
    }

    public CommandSwerveDrivetrain getDriveTrain() {
        return m_drivetrain;
    }

    public void clearAutoPreview() {}

    public void updateAutoPreviewActor() {}

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }

    public Pose2d getInitialPose() {
        // put onto the field, easier for simulation
        return new Pose2d(1, 1, Rotation2d.kZero);
    }    

    // public Command getDriveCommand() {
    //     // The controls are for field-oriented driving:
    //     // Left stick Y axis -> forward and backwards movement
    //     // Left stick X axis -> left and right movement
    //     // Right stick X axis -> rotation

    //     return m_drivetrain.applyRequest(() ->
    //             m_driveRequest.withVelocityX(-conditionAxis(m_driverController.getLeftY()) * MAX_SPEED)
    //                 .withVelocityY(-conditionAxis(m_driverController.getLeftX()) * MAX_SPEED)
    //                 .withRotationalRate(-conditionAxis(m_driverController.getRightX()) * MAX_ANGULAR_RATE)
    //             );
    // }

    // private double conditionAxis(double value) {
    //     value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);
    //     // Square the axis, retaining the sign
    //     return Math.abs(value) * value;
    // }

    private Command withHopperControl(Command shootCommand) {
        return shootCommand.alongWith(new PulseHopper(m_hopper, m_shooter, m_turret));
    }
}