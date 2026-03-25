// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.*;
import frc.robot.commands.autoCommands.AutoCommandInterface;
import frc.robot.commands.autoCommands.CoreAuto;
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
    private final CommandJoystick m_farm = new CommandJoystick(1);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagVision m_aprilTagVision = new AprilTagVision(Robot.RobotType.COMPBOT, m_logger.getField2d());
    private final ShooterFeeder m_shooterFeeder = new ShooterFeeder();
    private final Shooter m_shooter = new Shooter();
    private final Turret m_turret = new Turret(m_logger.getField2d());

    private final Intake m_intake = new Intake();
    private final Hopper m_hopper = new Hopper();

    private final InternalButton m_virtualShootButton = new InternalButton();

    // not used directly, but the periodic() method logs data
    @SuppressWarnings("unused")
    private final DataLogger m_dataLogger = new DataLogger();

    private final SendableChooser<String> m_chosenFieldSide = new SendableChooser<>();
    private final SendableChooser<List<Object>> m_chosenAutoPaths = new SendableChooser<>();

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

    // @Override
    // public void setDisableEnableValues(boolean enabled) {
    //     // Enabled = coast mode, Disabled = brake mode
    //     m_intake.getPivot().setBrakeMode(!enabled);
    // }

    private void configureAutos() {

        // assign the Shoot button that is used during Autos
        // used only when shooting directly in the command
        // not used by PathPlanner triggers
        m_virtualShootButton.whileTrue(getShootCommand());

        m_chosenAutoPaths.setDefaultOption("Depot Double Swipe Blitz", List.of(
                "First Swipe Blitz",
                "Swipe Shoot",
                "Depot Double Swipe Blitz",
                "Depot Trench Run Out"
                ));

        m_chosenAutoPaths.addOption("Triple Swipe Blitz", List.of(
                "First Swipe Blitz",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe",
                "Swipe Shoot Alt"
                ));

        // m_chosenAutoPaths.addOption("Shoot While Moving", List.of(   
        //         "Shoot While Moving"
        // ));

        SmartDashboard.putData("Auto Choice", m_chosenAutoPaths);

        m_chosenFieldSide.setDefaultOption("Depot Side", "Depot Side");
        m_chosenFieldSide.addOption("Outpost Side", "Outpost Side");
        SmartDashboard.putData("Field Side", m_chosenFieldSide);

        SmartDashboard.putBoolean("autoStatus/runningIntake", false);
        SmartDashboard.putBoolean("autoStatus/runningShooter", false);

        configureAutoEventTriggers();
    }

    public Command getShootCommand() {
        return new Shoot(m_shooter, m_turret, m_shooterFeeder, m_hopper, m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, ShotType.AUTO)
                        // .alongWith(
                        //     m_hopper.pulseCommand(),
                        //     new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true)));
    }
    
    private void configureAutoEventTriggers() {
        new EventTrigger("Run Intake").onTrue(m_intake.deployAndRollCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", true))));
        new EventTrigger("Stop Intake").onTrue(m_intake.stowCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", false))));

        new EventTrigger("Shooter Running").whileTrue(getShootCommand());
        new EventTrigger("Shooter Running").onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", false)));

     }

    private void configureBindings() {
        m_drivetrain.setDefaultCommand(getDriveCommand());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // enable/disable brake mode on the pivot when the robot is disabled
        RobotModeTriggers.disabled().onFalse(new InstantCommand(() -> m_intake.getPivot().setBrakeMode(false)));
        RobotModeTriggers.disabled().onTrue(
            new InstantCommand(() -> m_intake.getPivot().setBrakeMode(true)).ignoringDisable(true)
        );

        // Just shoot
        m_driverController.rightTrigger().whileTrue(getShootCommand());

        // shoot while intaking
        m_driverController.rightBumper().whileTrue(getShootCommand());
        m_driverController.rightBumper().onTrue(m_intake.getPivot().deployCommand());
        m_driverController.rightBumper().whileTrue(
                new StartEndCommand(m_intake.getRoller()::intake, m_intake.getRoller()::stop, m_intake.getRoller()));
                             
        // Deploy and run the intake (intake will stay out)
        m_driverController.leftTrigger().onTrue(m_intake.getPivot().deployCommand());
        m_driverController.leftTrigger().whileTrue(
                new StartEndCommand(m_intake.getRoller()::intake, m_intake.getRoller()::stop, m_intake.getRoller()));
                        // .alongWith(new StartEndCommand(m_hopper::intake, m_hopper::stop, m_hopper)));

        // Stow the intake
        m_driverController.leftBumper().onTrue(m_intake.stowCommand());

        // lock wheels
        m_driverController.back().whileTrue(m_drivetrain.applyRequest(() -> m_brakeRequest));

        // Unjam
        m_farm.button(21).whileTrue(UnJamCommand());
        m_driverController.a().whileTrue(UnJamHopperCommand());
        m_driverController.b().whileTrue(UnJamHopperCommand());
        m_driverController.y().whileTrue(UnJamHopperCommand());
        m_driverController.x().whileTrue(UnJamHopperCommand());

        // fixed shots - distance in inches, plus ROBOT angle of turret
        // ladder - robot against the outside of the ladder, intake to the left for the dirver
        m_farm.button(11).whileTrue(new Shoot(m_shooter, m_turret, m_shooterFeeder, m_hopper,
                    m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, 130.0, Rotation2d.kCCW_90deg)
                    .alongWith(m_hopper.pulseCommand()));

        // corner shot
        m_farm.button(13).whileTrue(new Shoot(m_shooter, m_turret, m_shooterFeeder, m_hopper,
                    m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, 210.0, Rotation2d.kZero)
                    .alongWith(m_hopper.pulseCommand()));

        m_farm.button(15).whileTrue(new Shoot(m_shooter, m_turret, m_shooterFeeder, m_hopper,
                    m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, ShotType.TEST)
                    .alongWith(m_hopper.pulseCommand()));

        m_farm.button(1).onTrue(new InstantCommand(m_shooter::increaseFlyFudge));
        m_farm.button(2).onTrue(new InstantCommand(m_shooter::decreaseFlyFudge));

        m_farm.button(24).onTrue(new InstantCommand(() -> m_intake.getPivot().fudgeIntakeOutWhenDeployed())); //TODO fix me

        m_farm.button(6).onTrue(new InstantCommand(m_shooter::increaseFeedFudge));
        m_farm.button(7).onTrue(new InstantCommand(m_shooter::decreaseFeedFudge));

        m_farm.button(9).onTrue(new InstantCommand(m_shooter::increaseHoodFudge));
        m_farm.button(10).onTrue(new InstantCommand(m_shooter::decreaseHoodFudge));

        m_farm.button(4).onTrue(new InstantCommand(m_turret::increaseTurretFudge));
        m_farm.button(5).onTrue(new InstantCommand(m_turret::decreaseTurretFudge));

        m_farm.button(12).onTrue(new InstantCommand(m_intake.getRoller()::increaseIntakeFudge));
        m_farm.button(14).onTrue(new InstantCommand(m_intake.getRoller()::decreaseIntakeFudge));

        // Reset the field-centric heading on Start press.
        m_driverController.start().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_driverController.back().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        // m_driverController.back().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        // m_driverController.start().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_driverController.start().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        m_drivetrain.registerTelemetry(m_logger::telemeterize);


        // *** Test Commands *** 

        // m_driverController.y().whileTrue(new StartEndCommand(()->m_shooter.getFlywheel().setRPM(3000.0), ()->m_shooter.getFlywheel().stop()));

        // m_driverController.x().onTrue(new InstantCommand(() -> m_shooter.getHood().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0)))));
        
        // SmartDashboard.putNumber("flywheel/testVoltage", 0.0); 
        // m_farm.button(22).onTrue(new InstantCommand(() -> m_shooter.getFlywheel().setVoltage(SmartDashboard.getNumber("flywheel/testVoltage", 0.0))));

        // m_farm.button(23).onTrue(new InstantCommand(() -> m_shooter.getFlywheel().setRPM(SmartDashboard.getNumber("flywheel/testRPM", 0.0))));

        SmartDashboard.putNumber("feeder/testVoltage", 0.0); 
        m_farm.button(22).onTrue(new InstantCommand(() -> m_shooterFeeder.setKickerVoltage(SmartDashboard.getNumber("feeder/testVoltage", 0.0))));

        m_farm.button(23).onTrue(new InstantCommand(() -> m_shooterFeeder.setKickerRPM(SmartDashboard.getNumber("kicker/testRPM", 0.0))));

        // m_driverController.a().onTrue(new InstantCommand(() -> m_shooterFeeder.setRPM(SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0))));

        // SmartDashboard.putNumber("turret/testAngle", 0.0);
        // m_farm.button(16).onTrue(new InstantCommand(() -> m_turret.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("turret/testAngle", 0.0)))));

        // m_farm.button(22).whileTrue(new InstantCommand(() -> m_turret.setAngle(Rotation2d.fromDegrees(320.0)))
        //         .andThen(new WaitCommand(0.4))
        //         .andThen(new InstantCommand(() -> m_turret.setAngle(Rotation2d.fromDegrees(280.0))))
        // );

        // Command turretAngleTest = new TMP_turretAngleTest(m_drivetrain::getPose, m_turret);
        // m_driverController.start().whileTrue(turretAngleTest);
        // SmartDashboard.putBoolean("TurretAngleTest", false);
        // Trigger turretAngleTestTrigger = new Trigger(() -> SmartDashboard.getBoolean("TurretAngleTest", false));
        // turretAngleTestTrigger.whileTrue(turretAngleTest);
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
                    m_chosenFieldSide.getSelected().equals("Outpost Side"), m_virtualShootButton);
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

    private Command UnJamCommand() {
        return new ParallelCommandGroup(
                new StartEndCommand(m_hopper::reverse, m_hopper::stop, m_hopper),
                new StartEndCommand(m_shooterFeeder::runReverseUnjam, m_shooterFeeder::stop, m_shooterFeeder),
                m_intake.outtakeCommand());
    }

    private Command UnJamHopperCommand() {
        return new ParallelCommandGroup(
                new StartEndCommand(m_hopper::reverse, m_hopper::stop, m_hopper),
                m_intake.outtakeCommand());
    }
}
