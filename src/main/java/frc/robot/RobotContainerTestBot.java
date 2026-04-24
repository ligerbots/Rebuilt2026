// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.autoCommands.AutoCommandInterface;
import frc.robot.commands.autoCommands.CoreAuto;
import frc.robot.generated.TunerConstantsTestBot;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainerTestBot extends RobotContainer {
    private record AutoPreviewData(List<Pose2d> poses, List<PathPlannerTrajectory> trajectories, double durationSec) {}

    private static double SPEED_LIMIT = 0.5;
    private double MAX_SPEED = SPEED_LIMIT * TunerConstantsTestBot.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
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
    private final AprilTagVision m_aprilTagVision = new AprilTagVision(Robot.RobotType.TESTBOT, m_logger.getField2d());

    private final SendableChooser<String> m_chosenFieldSide = new SendableChooser<>();
    private List<Pose2d> m_autoPreviewPoses = List.of();
    private List<PathPlannerTrajectory> m_autoPreviewTrajectories = List.of();
    private double m_autoPreviewDurationSec = 0.0;
    private double m_autoPreviewStartTimeSec = 0.0;
    private int m_autoSelectionCode = Integer.MIN_VALUE; 
    
    public RobotContainerTestBot() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        
        m_drivetrain = new CommandSwerveDrivetrain(
            m_aprilTagVision,
            TunerConstantsTestBot.DrivetrainConstants,
            TunerConstantsTestBot.FrontLeft, TunerConstantsTestBot.FrontRight, TunerConstantsTestBot.BackLeft, TunerConstantsTestBot.BackRight
        );

        m_drivetrain.setupPathPlanner();

        configureBindings();

        configureAutos();
    }

    private void configureAutos() {
        m_chosenFieldSide.setDefaultOption("Depot Side", "Depot Side");
        m_chosenFieldSide.addOption("Outpost Side", "Outpost Side");

        SmartDashboard.putData("Field Side", m_chosenFieldSide);
    }

    private void configureBindings() {
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

    public CommandSwerveDrivetrain getDriveTrain() {
        return m_drivetrain;
    }

    @Override
    public void clearAutoPreview() {
        m_logger.getField2d().getObject("selectedAutoPath").setPoses();
        m_logger.getField2d().getObject("selectedAutoActor").setPoses();
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
        String selectedFieldSide = m_chosenFieldSide.getSelected();
        int currentAutoSelectionCode = Objects.hash(selectedFieldSide,
            DriverStation.getAlliance());
        List<Object> pathFiles = List.of(
            "Start Bump to Fuel Begin",
            "Fuel Begin to Fuel End With Events",
            "Fuel End to Bump Finish With Events",
            "Bump Finish to Climb A"
        );

        // Only call constructor if the auto selection inputs have changed
        if (m_autoSelectionCode != currentAutoSelectionCode) {
            boolean isDepotSide = selectedFieldSide.equals("Depot Side");
            m_autoCommand = CoreAuto.getInstance(pathFiles, m_drivetrain, isDepotSide, null);
            AutoPreviewData autoPreview = buildAutoPreview(pathFiles, isDepotSide);
            m_autoPreviewPoses = autoPreview.poses();
            m_autoPreviewTrajectories = autoPreview.trajectories();
            m_autoPreviewDurationSec = autoPreview.durationSec();
            m_autoPreviewStartTimeSec = Timer.getFPGATimestamp();
            SmartDashboard.putString("Selected Auto", "TestBot Auto");
            m_logger.getField2d().getObject("selectedAutoPath").setPoses(m_autoPreviewPoses);
            updateAutoPreviewActor();
            m_autoSelectionCode = currentAutoSelectionCode;
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

    private AutoPreviewData buildAutoPreview(List<Object> pathSteps, boolean shouldMirrorPath) {
        List<Pose2d> previewPoses = new ArrayList<>();
        List<PathPlannerTrajectory> previewTrajectories = new ArrayList<>();
        double previewDurationSec = 0.0;
        RobotConfig robotConfig = loadAutoPreviewRobotConfig();

        for (Object step : pathSteps) {
            if (!(step instanceof String pathName)) {
                continue;
            }

            PathPlannerPath path = CommandSwerveDrivetrain.loadPath(pathName);
            if (path == null) {
                continue;
            }

            if (shouldMirrorPath) {
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
