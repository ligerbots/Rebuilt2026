// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.reflect.Field;
import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autoCommands.AutoCommandInterface;
import frc.robot.commands.autoCommands.CoreAuto;
import frc.robot.commands.ShootHub;
import frc.robot.commands.TMP_turretAngleTest;
import frc.robot.generated.TunerConstantsCompBot;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Shooter;
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

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    // private final CommandJoystick m_farm = new CommandJoystick(1);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagVision m_aprilTagVision = new AprilTagVision(Robot.RobotType.COMPBOT, m_logger.getField2d());
    private final Shooter m_shooter = new Shooter();
    private final ShooterFeeder m_shooterFeeder = new ShooterFeeder();
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
        new EventTrigger("Run Intake").onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", true)));
        new EventTrigger("Stop Intake").onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", false)));

        new EventTrigger("Run Shooter").onTrue(getTestingStartShootCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true))));
        new EventTrigger("Stop Shooter").onTrue(getTestingStopShootCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", false))));
    }

    private void configureBindings() {
        m_drivetrain.setDefaultCommand(getDriveCommand());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        m_driverController.leftTrigger().whileTrue(new StartEndCommand(m_intake.getIntakeRoller()::intake, m_intake.getIntakeRoller()::stop, m_intake.getIntakeRoller()));
        m_driverController.rightBumper().whileTrue(new StartEndCommand(m_hopper::run, m_hopper::stop, m_hopper));
        m_driverController.rightTrigger().whileTrue(new StartEndCommand(m_hopper::run, m_hopper::stop, m_hopper));

        SmartDashboard.putNumber("shooterFeeder/testRPM", 0.0); 
        m_driverController.leftTrigger().onTrue(new InstantCommand(() -> m_shooterFeeder.setRPM(SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0))));

        m_driverController.leftTrigger().onTrue(m_intake.getIntakePivot().deployCommand());
        m_driverController.leftBumper().onTrue(m_intake.retractIntakeCommand());

        SmartDashboard.putNumber("hood/testAngle", 0.0);
        SmartDashboard.putNumber("flywheel/testRPM", 0.0); 
        SmartDashboard.putNumber("shooterFeeder/testRPM", 0.0); 

        m_driverController.y().onTrue(
            new InstantCommand(() -> m_shooter.getFlywheel().setRPM(SmartDashboard.getNumber("flywheel/testRPM", 0.0)))
            .alongWith(
                new InstantCommand(() -> m_shooterFeeder.setRPM(SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0))),
                new InstantCommand(() -> m_shooter.getHood().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0))))
            )
        );
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

        // SmartDashboard.putNumber("intake/testVoltage", 0.0); 
        // SmartDashboard.putNumber("intake/testAngle", 0.0); 
        // SmartDashboard.putNumber("hopper/testVoltage", 0.0); 
        // m_driverController.leftBumper().whileTrue(new StartEndCommand(
        //     () -> m_intakeRoller.setVoltage(SmartDashboard.getNumber("intake/testVoltage", 0.0)), m_intakeRoller::stop, m_intakeRoller));
        // m_driverController.rightBumper().whileTrue(new StartEndCommand(
        //     () -> m_hopper.setVoltage(SmartDashboard.getNumber("hopper/testVoltage", 0.0)), m_hopper::stop, m_hopper));
        // m_driverController.b().onTrue(new InstantCommand(() -> m_intakePivot.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("intake/testAngle", 0.0)))));

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

        // m_driverController.y().onTrue(
        //     new InstantCommand(() -> m_shooter.getFlywheel().setRPM(SmartDashboard.getNumber("flywheel/testRPM", 0.0)))
        //     .alongWith(
        //         new InstantCommand(() -> m_shooterFeeder.setRPM(SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0))),
        //         new InstantCommand(() -> m_shooter.getHood().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0))))
        //     )
        // );
        // m_driverController.x().onTrue(new InstantCommand(m_shooter::stop).alongWith(new InstantCommand(m_shooterFeeder::stop)));
        
        // TODO: Enable me once shooter tabels are ready
        // new Trigger(this::shouldShootHub).whileTrue(new ShootHub(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose));
    }

    // Determines wether we should start shooting at the hub because we are in our zone.
    // private boolean shouldShootHub() {
    //     boolean onRedShouldHub = FieldConstants.isRedAlliance() && (m_drivetrain.getPose().getX() >= FieldConstants.FIELD_WIDTH - FieldConstants.SHOOT_HUB_LINE_BLUE);
    //     boolean onBlueShouldHub = !FieldConstants.isRedAlliance() && (m_drivetrain.getPose().getX() <= FieldConstants.SHOOT_HUB_LINE_BLUE);
    //     return onBlueShouldHub || onRedShouldHub;
    //     // m_driverController.y().onTrue(getTestingStartShootCommand());

    //     // m_driverController.x().onTrue(getTestingStopShootCommand());
    // }
    
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
            m_autoCommand = new CoreAuto(m_chosenAutoPaths.getSelected(), m_drivetrain,
                    m_chosenFieldSide.getSelected().equals("Depot Side"));
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
