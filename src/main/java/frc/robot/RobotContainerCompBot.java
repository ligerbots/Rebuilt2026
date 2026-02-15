// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.autoCommands.AutoCommandInterface;
import frc.robot.commands.autoCommands.CoreAuto;
import frc.robot.generated.TunerConstantsCompBot;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShotType;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.Turret;

public class RobotContainerCompBot extends RobotContainer {
    private static final double SPEED_LIMIT = 1.0;
    private double MAX_SPEED = SPEED_LIMIT * TunerConstantsCompBot.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private static final double JOYSTICK_DEADBAND = 0.05;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // set the swerve wheels in an X pattern
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry m_logger = new Telemetry(MAX_SPEED);

    private AutoCommandInterface m_autoCommand;
    // private Command m_autoCommand;

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    // private final CommandJoystick m_farm = new CommandJoystick(1);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagVision m_aprilTagVision = new AprilTagVision(Robot.RobotType.COMPBOT, m_logger.getField2d());
    private final ShooterFeeder m_shooterFeeder = new ShooterFeeder();
    private final Shooter m_shooter = new Shooter();
    private final Turret m_turret = new Turret();

    private final Intake m_intake = new Intake();
    private final Hopper m_hopper = new Hopper();

    private final SendableChooser<String> m_chosenFieldSide = new SendableChooser<>();
    private final SendableChooser<String[]> m_chosenAutoPaths = new SendableChooser<>();

    private int m_autoSelectionCode; 
    
    public RobotContainerCompBot() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        
        m_drivetrain = new CommandSwerveDrivetrain(
            m_aprilTagVision,
            TunerConstantsCompBot.DrivetrainConstants,
            TunerConstantsCompBot.FrontLeft, TunerConstantsCompBot.FrontRight, TunerConstantsCompBot.BackLeft, TunerConstantsCompBot.BackRight
        );

        m_drivetrain.setupPathPlanner();

        configureBindings();

        configureAutos();
    }

    private void configureAutos() {

        m_chosenAutoPaths.setDefaultOption("First Basic Auto", new String[] {
            "Start Bump to Fuel Begin",
            "Fuel Begin to Fuel End With Events",
            "Fuel End to Bump Finish With Events",
            "Bump Finish to Climb A"
        });

        m_chosenAutoPaths.addOption("Depot Simple", new String[] {
            "Depot Simple V1"
        });

        m_chosenAutoPaths.addOption("Depot Full Pass", new String[] {
            "Depot Full Pass Disrupt V2"
        });
        
        m_chosenAutoPaths.addOption("Drive Straight to Climb", new String[] {
            "Start Bump to Climb A"
        });
        
        SmartDashboard.putData("Auto Choice", m_chosenAutoPaths);

        m_chosenFieldSide.setDefaultOption("Depot Side", "Depot Side");
        m_chosenFieldSide.addOption("Outpost Side", "Outpost Side");
        SmartDashboard.putData("Field Side", m_chosenFieldSide);

        SmartDashboard.putBoolean("autoStatus/runningIntake", false);
        SmartDashboard.putBoolean("autoStatus/runningShooter", false);

        configureAutoEventTriggers();
    }

    private void configureAutoEventTriggers() {
        new EventTrigger("Run Intake").onTrue(m_intake.deployAndRollCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", true))));
        new EventTrigger("Stop Intake").onTrue(m_intake.stowCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", false))));

        // TODO: should need only 1 AUTO shot trigger
        new EventTrigger("Hub Shot Running").whileTrue(
            new Shoot(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose, ShotType.HUB)
                        .alongWith(
                            m_hopper.pulseCommand(),
                            new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true))));
        new EventTrigger("Hub Shot Running").onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", false)));

        new EventTrigger("Passing Shot Running").whileTrue(new Shoot(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose, ShotType.PASS)
                        .alongWith(
                            m_hopper.pulseCommand(),
                            new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true))));
        new EventTrigger("Passing Shot Running").onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", false)));
 
        // new EventTrigger("Run Shooter").onTrue(getTestingStartShootCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true))));
        // new EventTrigger("Stop Shooter").onTrue(getTestingStopShootCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", false))));
    }

    private void configureBindings() {
        m_drivetrain.setDefaultCommand(getDriveCommand());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // // shoot command - test version
        // m_driverController.rightTrigger().whileTrue(
        //     new StartEndCommand(
        //         () -> m_shooter.getFlywheel().setRPM(SmartDashboard.getNumber("flywheel/testRPM", 0.0)),
        //         m_shooter::stop)
        //     .alongWith(
        //             new StartEndCommand(
        //                 () -> m_shooterFeeder.setRPM(SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0)), 
        //                 m_shooterFeeder::stop),
        //             new StartEndCommand(
        //                 () -> m_shooter.getHood().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0))),
        //                 () -> m_shooter.getHood().setAngle(Rotation2d.kZero)),
        //             m_hopper.pulseCommand())
        // );
        // TEST Shot - allow shot calibratin
        m_driverController.rightTrigger().whileTrue(new Shoot(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose, ShotType.AUTO)
                        .alongWith(m_hopper.pulseCommand()));

        m_driverController.rightBumper().whileTrue(new Shoot(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose, ShotType.AUTO)
                        .alongWith(m_hopper.pulseCommand()));
        m_driverController.rightBumper().onTrue(m_intake.getPivot().deployCommand());
        m_driverController.rightBumper().whileTrue(
                new StartEndCommand(m_intake.getRoller()::intake, m_intake.getRoller()::stop, m_intake.getRoller()));
                             
        m_driverController.leftTrigger().onTrue(m_intake.getPivot().deployCommand());
        m_driverController.leftTrigger().whileTrue(
                new StartEndCommand(m_intake.getRoller()::intake, m_intake.getRoller()::stop, m_intake.getRoller())
                        .alongWith(new StartEndCommand(m_hopper::intake, m_hopper::stop, m_hopper)));

        m_driverController.leftBumper().onTrue(m_intake.stowCommand());

        m_driverController.a().whileTrue(new Shoot(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose, ShotType.TEST)
                        .alongWith(m_hopper.pulseCommand()));

        // SmartDashboard.putNumber("hood/testAngle", 0.0);
        // SmartDashboard.putNumber("flywheel/testRPM", 0.0); 
        // SmartDashboard.putNumber("shooterFeeder/testRPM", 0.0); 

        m_driverController.x().onTrue(new InstantCommand(m_shooter::stop).alongWith(new InstantCommand(m_shooterFeeder::stop)));

        // lock wheels
        // m_driverController.a().whileTrue(m_drivetrain.applyRequest(() -> m_brakeRequest));
        // m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_driverController.back().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        // m_driverController.back().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        // m_driverController.start().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_driverController.start().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        // m_driverController.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        m_drivetrain.registerTelemetry(m_logger::telemeterize);


        // *** Test Commands *** 
        
        // m_driverController.x().onTrue(new InstantCommand(() -> m_shooter.getHood().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0)))));
        
        // SmartDashboard.putNumber("flywheel/testVoltage", 0.0); 
        // m_driverController.y().onTrue(new InstantCommand(() -> m_shooter.getFlywheel().setVoltage(SmartDashboard.getNumber("flywheel/testVoltage", 0.0))));

        // m_driverController.b().onTrue(new InstantCommand(() -> m_shooter.getFlywheel().setRPM(SmartDashboard.getNumber("flywheel/testRPM", 0.0))));
        
        // m_driverController.a().onTrue(new InstantCommand(() -> m_shooterFeeder.setRPM(SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0))));

        // SmartDashboard.putNumber("turret/testAngle", 0.0);
        // m_driverController.a().onTrue(new InstantCommand(() -> m_turret.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("turret/testAngle", 0.0)))));

        Command turretAngleTest = new TMP_turretAngleTest(m_drivetrain::getPose, m_turret);
        m_driverController.start().whileTrue(turretAngleTest);
        SmartDashboard.putBoolean("TurretAngleTest", false);
        Trigger turretAngleTestTrigger = new Trigger(() -> SmartDashboard.getBoolean("TurretAngleTest", false));
        turretAngleTestTrigger.whileTrue(turretAngleTest);
    }

    private Command getTestingStartShootCommand() {
            return new InstantCommand(() -> m_shooter.getFlywheel().setRPM(SmartDashboard.getNumber("flywheel/testRPM", 0.0)))
            .alongWith(
                new InstantCommand(() -> m_shooterFeeder.setRPM(SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0))),
                new InstantCommand(() -> m_shooter.getHood().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0))))
            );
    }

    private Command getTestingStopShootCommand() {
            return new InstantCommand(m_shooter::stop).alongWith(new InstantCommand(m_shooterFeeder::stop));
    }

    public CommandSwerveDrivetrain getDriveTrain() {
        return m_drivetrain;
    }

    public Command getAutonomousCommand() {
        int currentAutoSelectionCode = Objects.hash(
            m_chosenAutoPaths.getSelected(),
            m_chosenFieldSide.getSelected(),
            DriverStation.getAlliance());
    
        // Only call constructor if the auto selection inputs have changed
        if (m_autoSelectionCode != currentAutoSelectionCode) {
            m_autoSelectionCode = currentAutoSelectionCode;
            m_autoCommand = CoreAuto.getInstance(m_chosenAutoPaths.getSelected(), m_drivetrain,
                    m_chosenFieldSide.getSelected().equals("Depot Side"));
            // m_autoCommand = new PathPlannerAuto(coreCommand);
        }
        return m_autoCommand;
    }

    public Pose2d getInitialPose() {
        return ((AutoCommandInterface) getAutonomousCommand()).getInitialPose();
    }    

    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation

        return m_drivetrain.applyRequest(() ->
                m_driveRequest.withVelocityX(-conditionAxis(m_driverController.getLeftY()) * MAX_SPEED)
                    .withVelocityY(-conditionAxis(m_driverController.getLeftX()) * MAX_SPEED)
                    .withRotationalRate(-conditionAxis(m_driverController.getRightX()) * MAX_ANGULAR_RATE)
            );
    }

    private double conditionAxis(double value) {
        value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);
        // Square the axis, retaining the sign
        return Math.abs(value) * value;
    }
}
