package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.generated.TunerConstantsTestBot.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    // values from 2024 competition. Maybe should be tuned
    private static final PIDConstants PATH_PLANNER_TRANSLATION_PID = new PIDConstants(5, 0, 0);
    private static final PIDConstants PATH_PLANNER_ANGLE_PID       = new PIDConstants(5, 0, 0);

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public Field2d m_field = new Field2d();
    private final AprilTagVision m_aprilTagVision;

    // private final SwerveDrivetrain m_swerveDrive;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        AprilTagVision aprilTagVision,
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        optimizeCAN();
        SmartDashboard.putData("Field", m_field);

        m_aprilTagVision = aprilTagVision;
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    // public CommandSwerveDrivetrain(
    //     AprilTagVision aprilTagVision,
    //     SwerveDrivetrainConstants drivetrainConstants,
    //     double odometryUpdateFrequency,
    //     SwerveModuleConstants<?, ?, ?>... modules
    // ) {
    //     super(drivetrainConstants, odometryUpdateFrequency, modules);
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }

    //     SmartDashboard.putData("Field", m_field);

    //     m_aprilTagVision = aprilTagVision;
    // }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    // public CommandSwerveDrivetrain(
    //     AprilTagVision aprilTagVision,
    //     SwerveDrivetrainConstants drivetrainConstants,
    //     double odometryUpdateFrequency,
    //     Matrix<N3, N1> odometryStandardDeviation,
    //     Matrix<N3, N1> visionStandardDeviation,
    //     SwerveModuleConstants<?, ?, ?>... modules
    // ) {
    //     super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }

    //     SmartDashboard.putData("Field", m_field);

    //     m_aprilTagVision = aprilTagVision;
    // }

    private void optimizeCAN() {
        DriverStation.reportWarning("Running Swerve CAN optimization", false);

        double odometryHz = getOdometryFrequency();
        DriverStation.reportWarning("Odometry frequency is " + odometryHz, false);

        for (var module : this.getModules()) {
            TalonFX driveMotor = module.getDriveMotor();
            TalonFX steerMotor = module.getSteerMotor();
            CANcoder steerEncoder = module.getEncoder();

            BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.ROBOT_FREQUENCY_HZ,
                driveMotor.getVelocity(),
                driveMotor.getMotorVoltage(),
                driveMotor.getStatorCurrent(),
                steerMotor.getVelocity(),
                steerMotor.getMotorVoltage(),
                steerMotor.getStatorCurrent(),
                steerEncoder.getAbsolutePosition()
            );
            
            BaseStatusSignal.setUpdateFrequencyForAll(
                odometryHz,
                driveMotor.getPosition(),
                steerMotor.getPosition()
            );

            ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor, steerEncoder);
        }

        // Also, do the Pigeon
        Pigeon2 pigeon = this.getPigeon2();
        pigeon.getYaw().setUpdateFrequency(odometryHz);
        pigeon.getAngularVelocityZWorld().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        pigeon.optimizeBusUtilization();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {

        m_field.setRobotPose(this.getState().Pose);

        if (RobotBase.isSimulation()) {
            m_aprilTagVision.updateSimulation(this);
        }

        m_aprilTagVision.addVisionMeasurements(this, m_field);

        // m_aprilTagVision

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Set the robot pose on the field - generally used to set start of Auto
     * @param pose  Pose of the robot, relative to Blue (0,0)
     */
    public void setPose(Pose2d pose) {
        this.resetPose(pose);
    }
    
    /**
     * Fetch the current robot pose on the field
     * @return Pose2d of the robot
     */
    public Pose2d getPose() {
        return  getState().Pose;
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is
     *         empty).
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public void setupPathPlanner() {
        try {
            // Load the RobotConfig from the settings file created by GUI. 
            // You should probably store this in your Constants file
            RobotConfig config = RobotConfig.fromGUISettings();

            // TODO: fix code to allow FF
            // final boolean enableFeedforward = true;
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    // Robot pose supplier
                    // this::getPose,
                    () -> this.getState().Pose,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    // this::setPose,
                    this::resetPose,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    // this::getRobotVelocity,
                    () -> this.getState().Speeds,
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                    // optionally outputs individual module feedforwards
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        // if (enableFeedforward) {
                        //     m_swerveDrive.drive(
                        //             speedsRobotRelative,
                        //             m_swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        //             moduleFeedForwards.linearForces());
                        // } else {
                        // this.set
                        // m_swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        this.setControl(autoRequest.withSpeeds(speedsRobotRelative)); // Consumer of ChassisSpeeds to drive the robot

                        // this.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(speedsRobotRelative.vxMetersPerSecond)
                        //     .withVelocityY(speedsRobotRelative.vxMetersPerSecond)
                        //     .withRotationalRate(speedsRobotRelative.omegaRadiansPerSecond));
                        //         // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
                        // }
                    },
                    // PPHolonomicController is the built in path following controller for holonomic
                    // drive trains
                    new PPHolonomicDriveController(
                            PATH_PLANNER_TRANSLATION_PID,
                            PATH_PLANNER_ANGLE_PID),
                    // The robot configuration
                    config,
                    // whether to flip directions for Red
                    // () -> FieldConstants.isRedAlliance(),
                    () -> FieldConstants.isRedAlliance(),
                    this
            // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    /**
     * Get the path follower with events.
     *
     * @param path PathPlanner path.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command followPath(PathPlannerPath path) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

    public Command pathFindToPose(Pose2d targetPose, PathConstraints constraints) {
        
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    public static PathPlannerPath loadPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return path;
        } catch (Exception e) {
            DriverStation.reportError(String.format("Unable to load PP path %s", pathName), true);
        }
        return null;
    }

}