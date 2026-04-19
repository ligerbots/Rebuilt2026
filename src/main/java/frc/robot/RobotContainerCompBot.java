// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
    private record AutoPreviewData(List<Pose2d> poses, List<PathPlannerTrajectory> trajectories, double durationSec) {}

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
    private final Hopper m_hopper;

    private final InternalButton m_virtualShootButton = new InternalButton();

    // not used directly, but the periodic() method logs data
    @SuppressWarnings("unused")
    private final DataLogger m_dataLogger = new DataLogger();

    private final SendableChooser<String> m_chosenFieldSide = new SendableChooser<>();
    private final SendableChooser<String> m_chosenAutoPaths = new SendableChooser<>();
    private final Map<String, List<Object>> m_autoPathOptions = new LinkedHashMap<>();
    private List<Pose2d> m_autoPreviewPoses = List.of();
    private List<PathPlannerTrajectory> m_autoPreviewTrajectories = List.of();
    private double m_autoPreviewDurationSec = 0.0;
    private double m_autoPreviewStartTimeSec = 0.0;

    private int m_autoSelectionCode = Integer.MIN_VALUE; 
    
    public RobotContainerCompBot() {
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

        configureAutos();
    }

    private void configureAutos() {

        // assign the Shoot button that is used during Autos
        // used only when shooting directly in the command
        // not used by PathPlanner triggers
        m_virtualShootButton.whileTrue(getShootCommand());

        addAutoOption("Depot Double Swipe Blitz", List.of(
                "First Swipe Blitz",
                "Swipe Shoot",
                "Depot Double Swipe Blitz",
                "Depot Trench Run Out"
                ), true);

        // "Depot Double Swipe Blitz.auto"
        // "Depot Double Swipe Steal.auto"
        // "Pass Blitz.auto"
        // "Swing Depot Double Swipe Blitz.auto"
        // "Triple Swipe Blitz.auto"
        // "Triple Swipe Steal.auto"

        addAutoOption("BStart Depot Bump Only", List.of(
                "Bump Preload Bump",
                "BStart First Swipe Bump",
                "Bump Bump Shoot",
                "BStart First Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("BStart Depot Double Bump", List.of(
                "Bump Preload Trench",
                "Second Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("BStart Depot Double Bump", List.of(
                "Bump Preload Trench",
                "Second Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("Depot Double Swipe Bump", List.of(
                "First Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("Depot Triple Swipe Bump", List.of(
                "First Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("Triple Swipe Bump", List.of(
                "First Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Trench Shoot"
                ));

        addAutoOption("Triple Swipe Blitz", List.of(
                "First Swipe Blitz",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe",
                "Swipe Shoot Alt"
                ));

        addAutoOption("Pass Blitz", List.of(
                "Pass Swipe",
                "Pass Shoot"
                ));
        
        addAutoOption("Center Auto", List.of(
                "Center to First Shoot",
                3.0, // shoot for 3 seconds to ensure all 8 balls are out
                "First Shoot to Depot"
                ));

        addAutoOption("Swing Depot Double Swipe Blitz", List.of(
                "Swing First Swipe Blitz",
                "Swipe Shoot",
                "Depot Double Swipe Blitz"
                ));

        addAutoOption("Steal DOUBLE Swipe", List.of(
                "First Swipe Steal",
                "Swipe Shoot",
                "Depot Double Swipe Blitz",
                "Depot Trench Run Out"
                ));

        addAutoOption("Steal TRIPLE Swipe", List.of(
                "First Swipe Steal",
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
        return withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, ShotType.AUTO));
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
                new StartEndCommand(m_intake.getRoller()::fastIntake, m_intake.getRoller()::stop, m_intake.getRoller()));
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
        m_farm.button(11).whileTrue(withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder,
                        m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, 130.0, Rotation2d.kCCW_90deg)));

        // corner shot
        m_farm.button(13).whileTrue(withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder,
                        m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, 210.0, Rotation2d.k180deg)));

        m_farm.button(15).whileTrue(withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder,
                        m_drivetrain::getPose, m_drivetrain::getFieldCentricSpeeds, ShotType.TEST)));

        m_farm.button(1).onTrue(new InstantCommand(m_shooter::increaseFlyFudge));
        m_farm.button(2).onTrue(new InstantCommand(m_shooter::decreaseFlyFudge));

        // set the intake sensor position assuming it is deployed
        m_farm.button(24).onTrue(new InstantCommand(() -> m_intake.getPivot().setPositionToDeployed()));

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

        // SmartDashboard.putNumber("feeder/testVoltage", 0.0); 
        // m_farm.button(22).onTrue(new InstantCommand(() -> m_shooterFeeder.setKickerVoltage(SmartDashboard.getNumber("feeder/testVoltage", 0.0))));

        // m_farm.button(23).onTrue(new InstantCommand(() -> m_shooterFeeder.setKickerRPM(SmartDashboard.getNumber("kicker/testRPM", 0.0))));

        // m_driverController.a().onTrue(new InstantCommand(() -> m_shooterFeeder.setRPM(SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0))));

        // SmartDashboard.putNumber("turret/testAngle", 0.0);
        // m_farm.button(22).onTrue(new InstantCommand(() -> m_turret.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("turret/testAngle", 0.0)))));

        // m_farm.button(23).whileTrue(
        //     new InstantCommand(() -> m_turret.setAngle(m_turret.getAngle().plus(Rotation2d.fromDegrees(4))))
        //         .andThen(new WaitCommand(0.018))
        //         .repeatedly()
        // );

        // m_farm.button(23).whileTrue(new InstantCommand(() -> m_turret.setAngle(Rotation2d.fromDegrees(320.0)))
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

    @Override
    public void clearAutoPreview() {
        m_logger.getField2d().getObject("selectedAutoPath").setPoses();
        m_logger.getField2d().getObject("selectedAutoActor").setPoses();
        // TODO: don't do this. Causes Auto to be recreated at start of Autonomous
        m_autoSelectionCode = Integer.MIN_VALUE;
    }

    @Override
    public void updateAutoPreviewActor() {
        Pose2d previewPose = getAnimatedPreviewPose();
        if (previewPose == null) {
            m_logger.getField2d().getObject("selectedAutoActor").setPoses();
            return;
        }

        m_logger.getField2d().getObject("selectedAutoActor").setPose(previewPose);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = m_chosenAutoPaths.getSelected();
        List<Object> selectedAutoPaths = m_autoPathOptions.get(selectedAutoName);
        String selectedFieldSide = m_chosenFieldSide.getSelected();
        int currentAutoSelectionCode = Objects.hash(
            selectedAutoName,
            selectedAutoPaths,  // TODO: get rid of this. Not clear it is needed, and is expensive
            selectedFieldSide,
            DriverStation.getAlliance());

        // Only call constructor if the auto selection inputs have changed
        if (m_autoSelectionCode != currentAutoSelectionCode) {
            double startT = Timer.getFPGATimestamp();

            m_autoSelectionCode = currentAutoSelectionCode;

            boolean isOutpostSide = selectedFieldSide.equals("Outpost Side");
            m_autoCommand = CoreAuto.getInstance(selectedAutoPaths, m_drivetrain, isOutpostSide, m_virtualShootButton);
            AutoPreviewData autoPreview = buildAutoPreview(selectedAutoPaths, isOutpostSide);
            m_autoPreviewPoses = autoPreview.poses();
            m_autoPreviewTrajectories = autoPreview.trajectories();
            m_autoPreviewDurationSec = autoPreview.durationSec();
            m_autoPreviewStartTimeSec = Timer.getFPGATimestamp();
            SmartDashboard.putString("Selected Auto", selectedAutoName);
            m_logger.getField2d().getObject("selectedAutoPath").setPoses(m_autoPreviewPoses);
            // TODO: remove this, maybe???
            updateAutoPreviewActor();

            System.out.println("*** Build Auto command took " + (Timer.getFPGATimestamp() - startT) + " seconds");
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

    private Command withHopperControl(Command shootCommand) {
        return shootCommand.alongWith(new PulseHopper(m_hopper, m_shooter, m_turret));
    }

    private AutoPreviewData buildAutoPreview(List<Object> pathSteps, boolean isOutpostSide) {
        List<Pose2d> previewPoses = new ArrayList<>();
        List<PathPlannerTrajectory> previewTrajectories = new ArrayList<>();
        double previewDurationSec = 0.0;

        if (pathSteps == null) {
            return new AutoPreviewData(previewPoses, previewTrajectories, previewDurationSec);
        }

        RobotConfig robotConfig = loadAutoPreviewRobotConfig();

        for (Object step : pathSteps) {
            if (!(step instanceof String pathName)) {
                continue;
            }

            PathPlannerPath path = CommandSwerveDrivetrain.loadPath(pathName);
            if (path == null) {
                continue;
            }

            if (isOutpostSide) {
                path = path.mirrorPath();
            }

            for (Pose2d pose : path.getPathPoses()) {
                previewPoses.add(FieldConstants.flipPose(pose));
            }

            if (robotConfig == null) {
                continue;
            }

            PathPlannerTrajectory trajectory = buildAutoPreviewTrajectory(path, robotConfig);
            if (trajectory == null) {
                continue;
            }

            previewTrajectories.add(trajectory);
            previewDurationSec += trajectory.getTotalTimeSeconds();
        }

        return new AutoPreviewData(previewPoses, previewTrajectories, previewDurationSec);
    }

    private void addAutoOption(String name, List<Object> pathSteps) {
        addAutoOption(name, pathSteps, false);
    }

    private void addAutoOption(String name, List<Object> pathSteps, boolean isDefault) {
        m_autoPathOptions.put(name, pathSteps);
        if (isDefault) {
            m_chosenAutoPaths.setDefaultOption(name, name);
        } else {
            m_chosenAutoPaths.addOption(name, name);
        }
    }

    private RobotConfig loadAutoPreviewRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            return null;
        }
    }

    private PathPlannerTrajectory buildAutoPreviewTrajectory(PathPlannerPath path, RobotConfig robotConfig) {
        Rotation2d startingRotation = getPreviewStartingRotation(path);
        double startingSpeedMps = path.getIdealStartingState() != null ? path.getIdealStartingState().velocityMPS() : 0.0;
        Rotation2d pathHeading = getPathHeading(path);
        Translation2d fieldVelocity = new Translation2d(startingSpeedMps, pathHeading);
        ChassisSpeeds startingSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldVelocity.getX(),
            fieldVelocity.getY(),
            0.0,
            startingRotation);

        return path.generateTrajectory(startingSpeeds, startingRotation, robotConfig);
    }

    private Rotation2d getPreviewStartingRotation(PathPlannerPath path) {
        if (path.getIdealStartingState() != null) {
            return path.getIdealStartingState().rotation();
        }

        return getPathHeading(path);
    }

    private Rotation2d getPathHeading(PathPlannerPath path) {
        List<Pose2d> pathPoses = path.getPathPoses();
        if (pathPoses.size() < 2) {
            return Rotation2d.kZero;
        }

        Translation2d headingVector = pathPoses.get(1).getTranslation().minus(pathPoses.get(0).getTranslation());
        if (headingVector.getNorm() < 1e-6) {
            return Rotation2d.kZero;
        }

        return headingVector.getAngle();
    }

    private Pose2d getAnimatedPreviewPose() {
        if (m_autoPreviewTrajectories.isEmpty()) {
            if (m_autoPreviewPoses.isEmpty()) {
                return null;
            }

            return m_autoPreviewPoses.get(0);
        }

        if (m_autoPreviewDurationSec <= 0.0) {
            return FieldConstants.flipPose(m_autoPreviewTrajectories.get(m_autoPreviewTrajectories.size() - 1).getEndState().pose);
        }

        double elapsedSec = Timer.getFPGATimestamp() - m_autoPreviewStartTimeSec;
        double previewTimeSec = elapsedSec % m_autoPreviewDurationSec;

        for (PathPlannerTrajectory trajectory : m_autoPreviewTrajectories) {
            double trajectoryDurationSec = trajectory.getTotalTimeSeconds();
            if (previewTimeSec <= trajectoryDurationSec) {
                return FieldConstants.flipPose(trajectory.sample(previewTimeSec).pose);
            }

            previewTimeSec -= trajectoryDurationSec;
        }

        return FieldConstants.flipPose(m_autoPreviewTrajectories.get(m_autoPreviewTrajectories.size() - 1).getEndState().pose);
    }
}
