// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.Shooter.ShotType;
import frc.robot.utilities.ShooterLookupTable.ShootValue;

/**
* Command that coordinates shooting at the hub target.
* Manages turret aiming, shooter spin-up, and feeder activation.
*/
public class Shoot extends Command {
    private static final String PLOT_SHOT_LOCATION_KEY = "shoot/plotShotVisualization"; // Shows the shot lines on the field view.
    private static final String STATE_KEY = "shoot/state"; // Shows which step the shoot command is in.
    private static final String STATE_TIME_SEC_KEY = "shoot/stateTimeSec"; // Shows how long we have been in the current step.
    private static final String MOVING_SHOT_TRANSLATION_LOOKAHEAD_SEC_DASHBOARD_KEY = "shoot/movingTranslationLatencySec"; // How far ahead to account for robot movement across the floor.
    private static final String MOVING_SHOT_ROTATION_LOOKAHEAD_SEC_DASHBOARD_KEY = "shoot/movingRotationLatencySec"; // How far ahead to account for robot turning.
    private static final String MOVING_SHOT_TIME_OF_FLIGHT_SCALE_DASHBOARD_KEY = "shoot/tofScale"; // Multiplies shot travel time for moving-shot aiming.
    private static final String NORMAL_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC_DASHBOARD_KEY = "shoot/kickerPullbackMinSec"; // How long to hold the kicker backward before spinning it up.
    private static final String NORMAL_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP_DASHBOARD_KEY = "shoot/kickerSpinupFlywheelCompScale"; // Extra flywheel boost while the kicker is coming up to speed.
    private static final String PASS_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC_DASHBOARD_KEY = "shoot/passKickerPullbackMinSec"; // Same pullback time, but for pass shots.
    private static final String PASS_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP_DASHBOARD_KEY = "shoot/passKickerSpinupFlywheelCompScale"; // Same flywheel boost, but for pass shots.
    private static final String CROSS_COMP_SIGNAL_DELAY_COMP_SEC_DASHBOARD_KEY = "shoot/crossCompLatencySec"; // How much signal delay to remove from CTRE speed readings.
    private static final String CROSS_COMP_FUEL_TRAVEL_LOOKAHEAD_SEC_DASHBOARD_KEY = "shoot/crossCompTransitLeadSec"; // Extra time to look ahead for the fuel traveling into the flywheel.
    private static final String FINISH_AFTER_RELEASE_NO_FUEL_SEC_DASHBOARD_KEY = "shoot/finishAfterReleaseNoFuelSec"; // After release, keep shooting until no fuel has passed for this long.
    private static final String CROSS_COMP_ACTIVE_KEY = "shoot/crossCompActive"; // True when cross-comp is changing the RPM commands.
    private static final String CROSS_COMP_APPLIED_TRANSIT_LEAD_SEC_KEY = "shoot/crossCompAppliedTransitLeadSec"; // Shows the look-ahead time cross-comp actually used this loop.
    private static final String NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE_DASHBOARD_KEY = "shoot/flywheelFromKickerDroopScale"; // How strongly kicker slowdown adds RPM to the flywheel on normal shots.
    private static final String NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE_DASHBOARD_KEY = "shoot/kickerFromFlywheelDroopScale"; // How strongly flywheel slowdown adds RPM to the kicker on normal shots.
    private static final String PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE_DASHBOARD_KEY = "shoot/passFlywheelFromKickerDroopScale"; // How strongly kicker slowdown adds RPM to the flywheel on pass shots.
    private static final String PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE_DASHBOARD_KEY = "shoot/passKickerFromFlywheelDroopScale"; // How strongly flywheel slowdown adds RPM to the kicker on pass shots.
    private static final String MAX_FLYWHEEL_CROSS_COMP_INCREASE_FRACTION_DASHBOARD_KEY = "shoot/flywheelCrossCompMaxFraction"; // The biggest flywheel RPM increase cross-comp is allowed to add.
    private static final String MAX_KICKER_CROSS_COMP_INCREASE_FRACTION_DASHBOARD_KEY = "shoot/kickerCrossCompMaxFraction"; // The biggest kicker RPM increase cross-comp is allowed to add.
    private static final String READY_TO_FEED_KEY = "shoot/readyToFeed"; // True when it is safe to move fuel into the shot.
    private static final String SHOT_ANGLE_KEY = "shoot/shotAngle"; // The turret angle we are trying to shoot at.
    private static final String TURRET_BLOCKED_KEY = "shoot/turretBlocked"; // True when the turret is jammed or in a blocked zone.
    private static final String TURRET_READY_KEY = "shoot/turretReady"; // True when the turret is clear and aimed correctly.
    private static final String CROSS_COMP_PREDICTED_FLYWHEEL_RPM_KEY = "shoot/crossCompPredictedFlywheelRPM"; // Estimated flywheel RPM when the fuel reaches it.
    private static final String CROSS_COMP_PREDICTED_KICKER_RPM_KEY = "shoot/crossCompPredictedKickerRPM"; // Estimated kicker RPM when the fuel reaches the flywheel.
    private static final String COMMANDED_FLYWHEEL_RPM_KEY = "shoot/commandedFlywheelRPM"; // Final flywheel target after all adjustments.
    private static final String COMMANDED_KICKER_RPM_KEY = "shoot/commandedKickerRPM"; // Final kicker target after all adjustments.
    private static final String KICKER_DROOP_FRACTION_LOG_KEY = "shoot/kickerDroopFraction"; // How far below goal the kicker is predicted to be.
    private static final String FLYWHEEL_DROOP_FRACTION_LOG_KEY = "shoot/flywheelDroopFraction"; // How far below goal the flywheel is predicted to be.
    private static final String EFFECTIVE_SHOT_TYPE_KEY = "shoot/effectiveShotType"; // Which shot style we ended up using.
    private static final String TARGET_X_KEY = "shoot/targetX"; // Target X position used by moving-shot aiming.
    private static final String ROBOT_VEL_KEY = "shoot/robotVel"; // Robot drive speed used by moving-shot aiming.
    private static final String ROBOT_OMEGA_KEY = "shoot/robotOmega"; // Robot turn speed used by moving-shot aiming.
    private static final String SHOT_TOF_KEY = "shoot/tof"; // Shot travel time used by moving-shot aiming.
    private static final String TARGET_DISTANCE_KEY = "shoot/targetDistance"; // Distance used to look up shot values.
    private static final String TEST_HOOD_ANGLE_KEY = "hood/testAngle"; // Manual hood angle for test shots.
    private static final String TEST_FLYWHEEL_RPM_KEY = "flywheel/testRPM"; // Manual flywheel RPM for test shots.
    private static final String TEST_KICKER_RPM_KEY = "kicker/testRPM"; // Manual kicker RPM for test shots.
    private static final String TEST_TIME_OF_FLIGHT_KEY = "shooter/testTimeOfFlight"; // Manual shot travel time for test shots.

    // Feature toggle dashboard keys.
    private static final String PHASED_SHOOT_CONTROL_ENABLED_DASHBOARD_KEY = "shoot/phasedControlEnabled"; // Turns the new multi-step shoot flow on or off.
    private static final String PULL_BACK_KICKER_WHEN_TURRET_OFF_TARGET_ENABLED_DASHBOARD_KEY = "shoot/pullBackKickerWhenTurretOffTargetEnabled"; // If true, pull fuel away from the flywheel when the turret is not ready.
    private static final String FINISH_AFTER_RELEASE_ENABLED_DASHBOARD_KEY = "shoot/finishAfterReleaseEnabled"; // If true, keep shooting briefly after button release to finish the current burst.
    private static final String NORMAL_SHOT_CROSS_COMP_ENABLED_DASHBOARD_KEY = "shoot/crossCompEnabled"; // Turns on cross-comp for normal shots.
    private static final String NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED_DASHBOARD_KEY = "shoot/flywheelFromKickerCompEnabled"; // Lets kicker slowdown raise the flywheel command on normal shots.
    private static final String NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED_DASHBOARD_KEY = "shoot/kickerFromFlywheelCompEnabled"; // Lets flywheel slowdown raise the kicker command on normal shots.
    private static final String PASS_SHOT_CROSS_COMP_ENABLED_DASHBOARD_KEY = "shoot/passCrossCompEnabled"; // Turns on cross-comp for pass shots.
    private static final String PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED_DASHBOARD_KEY = "shoot/passFlywheelFromKickerCompEnabled"; // Lets kicker slowdown raise the flywheel command on pass shots.
    private static final String PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED_DASHBOARD_KEY = "shoot/passKickerFromFlywheelCompEnabled"; // Lets flywheel slowdown raise the kicker command on pass shots.

    private static final double DEFAULT_NORMAL_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC = 0.15; // Start by holding the kicker backward for 0.15 seconds on normal shots.
    private static final double DEFAULT_NORMAL_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP = 1.05; // Raise flywheel target by 5% while the kicker spins up on normal shots.
    private static final double DEFAULT_PASS_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC = 0.08; // Hold the kicker backward for a shorter 0.08 seconds on pass shots.
    private static final double DEFAULT_PASS_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP = 1.02; // Raise flywheel target by 2% while the kicker spins up on pass shots.
    private static final double DEFAULT_CROSS_COMP_SIGNAL_DELAY_COMP_SEC = 0.08; // Treat speed readings as up to 0.08 seconds old and correct for that delay.
    private static final double DEFAULT_CROSS_COMP_FUEL_TRAVEL_LOOKAHEAD_SEC = 0.07; // Look another 0.07 seconds ahead for when the fuel reaches the flywheel.
    private static final double DEFAULT_FINISH_AFTER_RELEASE_NO_FUEL_SEC = 0.60; // After release, wait 0.60 seconds with no fuel passing before ending the shot.
    private static final double DEFAULT_NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE = 0.75; // If the kicker is slowing down, add 75% of that slowdown into the flywheel target on normal shots.
    private static final double DEFAULT_NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE = 0.35; // If the flywheel is slowing down, add 35% of that slowdown into the kicker target on normal shots.
    private static final double DEFAULT_PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE = 0.15; // If the kicker is slowing down, add 15% of that slowdown into the flywheel target on pass shots.
    private static final double DEFAULT_PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE = 0.05; // If the flywheel is slowing down, add 5% of that slowdown into the kicker target on pass shots.
    private static final double DEFAULT_MAX_FLYWHEEL_CROSS_COMP_INCREASE_FRACTION = 0.12; // Never let cross-comp raise the flywheel more than 12% above its base target.
    private static final double DEFAULT_MAX_KICKER_CROSS_COMP_INCREASE_FRACTION = 0.08; // Never let cross-comp raise the kicker more than 8% above its base target.

    // Feature toggle defaults.
    private static final boolean DEFAULT_PHASED_SHOOT_CONTROL_ENABLED = false; // Leave the new multi-step shoot flow off at startup.
    private static final boolean DEFAULT_PULL_BACK_KICKER_WHEN_TURRET_OFF_TARGET_ENABLED = false; // Leave turret-off-target kicker pullback off at startup.
    private static final boolean DEFAULT_FINISH_AFTER_RELEASE_ENABLED = false; // Leave finish-after-release shooting off at startup.
    private static final boolean DEFAULT_NORMAL_SHOT_CROSS_COMP_ENABLED = false; // Leave normal-shot cross-comp off at startup.
    private static final boolean DEFAULT_NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED = false; // Leave kicker-to-flywheel help off at startup.
    private static final boolean DEFAULT_NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED = false; // Leave flywheel-to-kicker help off at startup.
    private static final boolean DEFAULT_PASS_SHOT_CROSS_COMP_ENABLED = false; // Leave pass-shot cross-comp off at startup.
    private static final boolean DEFAULT_PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED = false; // Leave pass kicker-to-flywheel help off at startup.
    private static final boolean DEFAULT_PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED = false; // Leave pass flywheel-to-kicker help off at startup.

    private static record ShotSelection(Translation2d target, ShotType effectiveShotType) {}
    private static record CrossCompTargets(double flywheelRPM, double kickerRPM) {}

    private enum ShootState {
        SPINNING_UP,
        SETTLING_KICKER,
        FEEDING
    }

    private enum PassSide {
        LEFT,
        RIGHT
    }

    private final Shooter m_shooter;
    private final Turret m_turret;
    private final ShooterFeeder m_feeder;
    private final Supplier<ChassisSpeeds> m_speedsSupplier;
    private final Supplier<Pose2d> m_poseSupplier;
    private final BooleanSupplier m_wantsShootSupplier;

    private final Shooter.ShotType m_shotType;

    private static final double DEFAULT_MOVING_SHOT_TRANSLATION_LOOKAHEAD_SEC = 0.05; // Look 0.05 seconds ahead for robot movement across the floor.
    private static final double DEFAULT_MOVING_SHOT_ROTATION_LOOKAHEAD_SEC = 0.05; // Look 0.05 seconds ahead for robot turning.
    private static final double DEFAULT_MOVING_SHOT_TIME_OF_FLIGHT_SCALE = 0.75; // Use 75% of the raw shot travel time from the lookup table.

    private static final double FLYWHEEL_SCALE = 1.0;

    // for fixed shot only
    private final Translation2d m_fixedShotVector;

    private ShootState m_shootState = ShootState.SPINNING_UP;
    private double m_stateStartTimeSec = 0.0;
    private PassSide m_latchedPassSide = null;
    private boolean m_readyToFeed = false;

    private Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, Shooter.ShotType shotType,
            double shotDistanceInches, Rotation2d turretHeading, BooleanSupplier wantsShootSupplier) {
        m_turret = turret;
        m_shooter = shooter;
        m_feeder = feeder;
        addRequirements(shooter, turret, feeder);
        
        m_poseSupplier = poseSupplier;
        m_speedsSupplier = speeds;

        m_shotType = shotType;
        m_wantsShootSupplier = wantsShootSupplier;

        // fixed shot only
        m_fixedShotVector = new Translation2d(Units.inchesToMeters(shotDistanceInches), turretHeading);

        // SD values used in the Test command
        SmartDashboard.putNumber(TEST_HOOD_ANGLE_KEY, 0.0);
        SmartDashboard.putNumber(TEST_FLYWHEEL_RPM_KEY, 0.0); 
        SmartDashboard.putNumber(TEST_KICKER_RPM_KEY, 0.0); 
        SmartDashboard.setDefaultBoolean(PLOT_SHOT_LOCATION_KEY, RobotBase.isSimulation());
        SmartDashboard.setDefaultNumber(MOVING_SHOT_TRANSLATION_LOOKAHEAD_SEC_DASHBOARD_KEY, DEFAULT_MOVING_SHOT_TRANSLATION_LOOKAHEAD_SEC);
        SmartDashboard.setDefaultNumber(MOVING_SHOT_ROTATION_LOOKAHEAD_SEC_DASHBOARD_KEY, DEFAULT_MOVING_SHOT_ROTATION_LOOKAHEAD_SEC);
        SmartDashboard.setDefaultNumber(MOVING_SHOT_TIME_OF_FLIGHT_SCALE_DASHBOARD_KEY, DEFAULT_MOVING_SHOT_TIME_OF_FLIGHT_SCALE);
        SmartDashboard.setDefaultNumber(NORMAL_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC_DASHBOARD_KEY, DEFAULT_NORMAL_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC);
        SmartDashboard.setDefaultNumber(NORMAL_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP_DASHBOARD_KEY, DEFAULT_NORMAL_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP);
        SmartDashboard.setDefaultNumber(PASS_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC_DASHBOARD_KEY, DEFAULT_PASS_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC);
        SmartDashboard.setDefaultNumber(PASS_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP_DASHBOARD_KEY, DEFAULT_PASS_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP);
        SmartDashboard.setDefaultNumber(CROSS_COMP_SIGNAL_DELAY_COMP_SEC_DASHBOARD_KEY, DEFAULT_CROSS_COMP_SIGNAL_DELAY_COMP_SEC);
        SmartDashboard.setDefaultNumber(CROSS_COMP_FUEL_TRAVEL_LOOKAHEAD_SEC_DASHBOARD_KEY, DEFAULT_CROSS_COMP_FUEL_TRAVEL_LOOKAHEAD_SEC);
        SmartDashboard.setDefaultNumber(FINISH_AFTER_RELEASE_NO_FUEL_SEC_DASHBOARD_KEY, DEFAULT_FINISH_AFTER_RELEASE_NO_FUEL_SEC);
        SmartDashboard.setDefaultNumber(NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE_DASHBOARD_KEY, DEFAULT_NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE);
        SmartDashboard.setDefaultNumber(NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE_DASHBOARD_KEY, DEFAULT_NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE);
        SmartDashboard.setDefaultNumber(PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE_DASHBOARD_KEY, DEFAULT_PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE);
        SmartDashboard.setDefaultNumber(PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE_DASHBOARD_KEY, DEFAULT_PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE);
        SmartDashboard.setDefaultNumber(MAX_FLYWHEEL_CROSS_COMP_INCREASE_FRACTION_DASHBOARD_KEY, DEFAULT_MAX_FLYWHEEL_CROSS_COMP_INCREASE_FRACTION);
        SmartDashboard.setDefaultNumber(MAX_KICKER_CROSS_COMP_INCREASE_FRACTION_DASHBOARD_KEY, DEFAULT_MAX_KICKER_CROSS_COMP_INCREASE_FRACTION);

        // Feature toggles live together here so they are easy to find.
        SmartDashboard.setDefaultBoolean(PHASED_SHOOT_CONTROL_ENABLED_DASHBOARD_KEY, DEFAULT_PHASED_SHOOT_CONTROL_ENABLED);
        SmartDashboard.setDefaultBoolean(
                PULL_BACK_KICKER_WHEN_TURRET_OFF_TARGET_ENABLED_DASHBOARD_KEY,
                DEFAULT_PULL_BACK_KICKER_WHEN_TURRET_OFF_TARGET_ENABLED);
        SmartDashboard.setDefaultBoolean(
                FINISH_AFTER_RELEASE_ENABLED_DASHBOARD_KEY,
                DEFAULT_FINISH_AFTER_RELEASE_ENABLED);
        SmartDashboard.setDefaultBoolean(NORMAL_SHOT_CROSS_COMP_ENABLED_DASHBOARD_KEY, DEFAULT_NORMAL_SHOT_CROSS_COMP_ENABLED);
        SmartDashboard.setDefaultBoolean(NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED_DASHBOARD_KEY, DEFAULT_NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED);
        SmartDashboard.setDefaultBoolean(NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED_DASHBOARD_KEY, DEFAULT_NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED);
        SmartDashboard.setDefaultBoolean(PASS_SHOT_CROSS_COMP_ENABLED_DASHBOARD_KEY, DEFAULT_PASS_SHOT_CROSS_COMP_ENABLED);
        SmartDashboard.setDefaultBoolean(
                PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED_DASHBOARD_KEY,
                DEFAULT_PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED);
        SmartDashboard.setDefaultBoolean(
                PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED_DASHBOARD_KEY,
                DEFAULT_PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED);
    }

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, Shooter.ShotType shotType) {
        this(shooter, turret, feeder,
                poseSupplier, speeds, shotType, 0.0, Rotation2d.kZero, null);
    }

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, Shooter.ShotType shotType,
            BooleanSupplier wantsShootSupplier) {
        this(shooter, turret, feeder,
                poseSupplier, speeds, shotType, 0.0, Rotation2d.kZero, wantsShootSupplier);
    }

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder,
                Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, 
                 double shotDistanceInches, Rotation2d turretHeading) {
        this(shooter, turret, feeder,
                poseSupplier, speeds, ShotType.FIXED, shotDistanceInches, turretHeading, null);
    }

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder,
                Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds,
                 double shotDistanceInches, Rotation2d turretHeading, BooleanSupplier wantsShootSupplier) {
        this(shooter, turret, feeder,
                poseSupplier, speeds, ShotType.FIXED, shotDistanceInches, turretHeading, wantsShootSupplier);
    }

    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_latchedPassSide = null;
        m_shootState = ShootState.SPINNING_UP;
        m_stateStartTimeSec = Timer.getFPGATimestamp();
        m_readyToFeed = false;
        m_shooter.getFlywheel().setJamWatchActive(false);
    }
    
    @Override
    public void execute() {
        // Pose is needed for plotting, so fetch it once here
        Pose2d robotPose = m_poseSupplier.get();

        ShotType effectiveShotType;
        Translation2d shotVector;
        if (m_shotType == ShotType.FIXED) {
            shotVector = m_fixedShotVector;
            effectiveShotType = ShotType.HUB;
            
            if (shouldPlotShotLocation()) {
                m_turret.plotShotVectors(robotPose, shotVector, Translation2d.kZero, Translation2d.kZero);
            }
        } else {    
            ShotSelection shotSelection = targetForShotType(robotPose);
            effectiveShotType = shotSelection.effectiveShotType();
            shotVector = findMovingShotVector(robotPose, shotSelection.target(), effectiveShotType);
            // old static shot
            // Translation2d translationToTarget = Turret.getTranslationToGoal(robotPose, target);
        }

        ShootValue shotValue;
        if (m_shotType == ShotType.TEST) {
            shotValue = testShotValue();
        } else {
            shotValue = m_shooter.getShootValue(shotLookupDistance(robotPose, shotVector, effectiveShotType), effectiveShotType);
        }
        
        Rotation2d angle = shotVector.getAngle();
        SmartDashboard.putNumber(SHOT_ANGLE_KEY, angle.getDegrees());

        m_turret.setAngle(angle);

        boolean turretBlocked = m_turret.isRecovering() || m_turret.inDeadZone();
        boolean turretReady = !turretBlocked && m_turret.isOnTarget();
        boolean shooterReady = m_shooter.onTarget();
        boolean kickerReady = m_feeder.isKickerOnTarget();
        boolean wantsShoot = wantsShoot();
        double kickerPullbackMinSec = getKickerPullbackMinSec(effectiveShotType);
        double kickerSpinupFlywheelCompScale = getKickerSpinupFlywheelCompScale(effectiveShotType);

        SmartDashboard.putBoolean(TURRET_BLOCKED_KEY, turretBlocked);
        SmartDashboard.putBoolean(TURRET_READY_KEY, turretReady);

        if (isPhasedControlEnabled()) {
            updateShootState(turretReady, shooterReady, kickerReady, kickerPullbackMinSec, wantsShoot);
            applyStateOutputs(shotValue, effectiveShotType, kickerSpinupFlywheelCompScale, turretReady);
            m_readyToFeed = m_shootState == ShootState.FEEDING
                    && turretReady
                    && shooterReady
                    && kickerReady;
        } else {
            applyLegacyOutputs(shotValue, turretReady, shooterReady);
        }
        SmartDashboard.putBoolean(READY_TO_FEED_KEY, m_readyToFeed);

        if (!shouldPlotShotLocation()) {
            m_turret.clearShotVisualization();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
        m_shooter.getFlywheel().setJamWatchActive(false);
        m_readyToFeed = false;
        SmartDashboard.putBoolean(READY_TO_FEED_KEY, false);

        // erase our velocity vector scribblings
        if (shouldPlotShotLocation()) {
            m_turret.clearShotVisualization();
        }
    }
    
    /**
    * Determines when the shoot command should end.
    * 
    * @return true when the command should terminate (currently stubbed)
    */
    @Override
    public boolean isFinished() {
        if (m_wantsShootSupplier == null || wantsShoot()) {
            return false;
        }

        if (!isFinishAfterReleaseEnabled()) {
            return true;
        }

        if (isPhasedControlEnabled()) {
            if (m_shootState != ShootState.FEEDING) {
                return true;
            }
        } else if (!m_readyToFeed) {
            return true;
        }

        return m_shooter.getFlywheel().getTimeSinceLastFuelPassSec() >= getFinishAfterReleaseNoFuelSec();
    }

    public boolean isReadyToFeed() {
        return m_readyToFeed;
    }

    private void updateShootState(
            boolean turretReady,
            boolean shooterReady,
            boolean kickerReady,
            double kickerPullbackMinSec,
            boolean wantsShoot) {
        switch (m_shootState) {
            case SPINNING_UP:
                if (wantsShoot && timeInState() >= kickerPullbackMinSec && shooterReady) {
                    setShootState(ShootState.SETTLING_KICKER);
                }
                break;
            case SETTLING_KICKER:
                if (wantsShoot && turretReady && shooterReady && kickerReady) {
                    setShootState(ShootState.FEEDING);
                }
                break;
            case FEEDING:
                if (!turretReady || !shooterReady || !kickerReady) {
                    setShootState(ShootState.SETTLING_KICKER);
                }
                break;
            default:
                break;
        }
    }

    private void applyStateOutputs(
            ShootValue baseShotValue,
            ShotType effectiveShotType,
            double kickerSpinupFlywheelCompScale,
            boolean turretReady) {
        ShootValue commandedShotValue = new ShootValue(baseShotValue);
        commandedShotValue.flyRPM *= FLYWHEEL_SCALE;

        switch (m_shootState) {
            case SPINNING_UP:
                m_shooter.setShootValues(commandedShotValue);
                m_feeder.pullBackKicker();
                m_feeder.stopFeederBelts();
                m_shooter.getFlywheel().setJamWatchActive(false);
                logCrossComp(commandedShotValue.flyRPM, m_feeder.getKickerGoalRPM(), 0.0, 0.0);
                break;
            case SETTLING_KICKER:
                commandedShotValue.flyRPM *= kickerSpinupFlywheelCompScale;
                m_shooter.setShootValues(commandedShotValue, false);
                m_feeder.stopFeederBelts();
                if (turretReady) {
                    CrossCompTargets settlingTargets =
                            applyCrossCompensation(effectiveShotType, commandedShotValue.flyRPM, baseShotValue.feedRPM);
                    commandedShotValue.flyRPM = settlingTargets.flywheelRPM();
                    m_shooter.setShootValues(commandedShotValue, false);
                    m_feeder.setKickerRPM(settlingTargets.kickerRPM());
                } else if (isTurretOffTargetKickerPullbackEnabled()) {
                    m_feeder.pullBackKicker();
                    logCrossComp(commandedShotValue.flyRPM, m_feeder.getKickerGoalRPM(), 0.0, 0.0);
                } else {
                    CrossCompTargets settlingTargets =
                            applyCrossCompensation(effectiveShotType, commandedShotValue.flyRPM, baseShotValue.feedRPM);
                    commandedShotValue.flyRPM = settlingTargets.flywheelRPM();
                    m_shooter.setShootValues(commandedShotValue, false);
                    m_feeder.setKickerRPM(settlingTargets.kickerRPM());
                }
                m_shooter.getFlywheel().setJamWatchActive(false);
                break;
            case FEEDING:
                CrossCompTargets feedingTargets =
                        applyCrossCompensation(effectiveShotType, commandedShotValue.flyRPM, baseShotValue.feedRPM);
                commandedShotValue.flyRPM = feedingTargets.flywheelRPM();
                m_shooter.setShootValues(commandedShotValue, false);
                m_feeder.setKickerRPM(feedingTargets.kickerRPM());
                m_feeder.runFeederBelts();
                m_shooter.getFlywheel().setJamWatchActive(true);
                break;
            default:
                break;
        }

        SmartDashboard.putString(STATE_KEY, m_shootState.toString());
        SmartDashboard.putNumber(STATE_TIME_SEC_KEY, timeInState());
    }

    private void applyLegacyOutputs(ShootValue baseShotValue, boolean turretReady, boolean shooterReady) {
        ShootValue commandedShotValue = new ShootValue(baseShotValue);
        commandedShotValue.flyRPM *= FLYWHEEL_SCALE;

        m_shooter.setShootValues(commandedShotValue);
        m_readyToFeed = turretReady && shooterReady;

        if (m_readyToFeed) {
            m_feeder.setKickerRPM(baseShotValue.feedRPM);
            m_feeder.runFeederBelts();
            m_shooter.getFlywheel().setJamWatchActive(true);
        } else {
            if (isTurretOffTargetKickerPullbackEnabled()) {
                m_feeder.pullBackKicker();
            } else {
                m_feeder.setKickerRPM(baseShotValue.feedRPM);
            }
            m_feeder.stopFeederBelts();
            m_shooter.getFlywheel().setJamWatchActive(false);
        }

        SmartDashboard.putBoolean(CROSS_COMP_ACTIVE_KEY, false);
        SmartDashboard.putNumber(CROSS_COMP_PREDICTED_FLYWHEEL_RPM_KEY, m_shooter.getFlywheel().getRPM());
        SmartDashboard.putNumber(CROSS_COMP_PREDICTED_KICKER_RPM_KEY, m_feeder.getKickerRPM());
        SmartDashboard.putNumber(CROSS_COMP_APPLIED_TRANSIT_LEAD_SEC_KEY, 0.0);
        logCrossComp(commandedShotValue.flyRPM, baseShotValue.feedRPM, 0.0, 0.0);
        SmartDashboard.putString(STATE_KEY, "LEGACY");
        SmartDashboard.putNumber(STATE_TIME_SEC_KEY, 0.0);
    }

    private CrossCompTargets applyCrossCompensation(
            ShotType effectiveShotType,
            double baseFlywheelRPM,
            double baseKickerRPM) {
        if (baseFlywheelRPM <= 0.0 || baseKickerRPM <= 0.0) {
            logCrossComp(baseFlywheelRPM, baseKickerRPM, 0.0, 0.0);
            return new CrossCompTargets(baseFlywheelRPM, baseKickerRPM);
        }

        if (!isCrossCompEnabled(effectiveShotType)) {
            logCrossComp(baseFlywheelRPM, baseKickerRPM, 0.0, 0.0);
            SmartDashboard.putBoolean(CROSS_COMP_ACTIVE_KEY, false);
            SmartDashboard.putNumber(CROSS_COMP_PREDICTED_FLYWHEEL_RPM_KEY, m_shooter.getFlywheel().getRPM());
            SmartDashboard.putNumber(CROSS_COMP_PREDICTED_KICKER_RPM_KEY, m_feeder.getKickerRPM());
            SmartDashboard.putNumber(CROSS_COMP_APPLIED_TRANSIT_LEAD_SEC_KEY, 0.0);
            return new CrossCompTargets(baseFlywheelRPM, baseKickerRPM);
        }

        double maxLatencySec = getCrossCompLatencySec();
        double transitLeadSec = getCrossCompTransitLeadSec();
        double predictedFlywheelRPM = m_shooter.getFlywheel().getLatencyCompensatedRPM(maxLatencySec)
                + m_shooter.getFlywheel().getAccelerationRPMPerSec() * transitLeadSec;
        double predictedKickerRPM = m_feeder.getLatencyCompensatedKickerRPM(maxLatencySec)
                + m_feeder.getKickerAccelerationRPMPerSec() * transitLeadSec;

        double flywheelDroopFraction = MathUtil.clamp(
                (baseFlywheelRPM - predictedFlywheelRPM) / Math.max(baseFlywheelRPM, 1.0),
                0.0,
                getFlywheelCrossCompMaxFraction());
        double kickerDroopFraction = MathUtil.clamp(
                (baseKickerRPM - predictedKickerRPM) / Math.max(baseKickerRPM, 1.0),
                0.0,
                getKickerCrossCompMaxFraction());

        double flywheelFromKickerScale = getFlywheelFromKickerScale(effectiveShotType);
        double kickerFromFlywheelScale = getKickerFromFlywheelScale(effectiveShotType);
        if (!isFlywheelFromKickerCompEnabled(effectiveShotType)) {
            flywheelFromKickerScale = 0.0;
        }
        if (!isKickerFromFlywheelCompEnabled(effectiveShotType)) {
            kickerFromFlywheelScale = 0.0;
        }

        double flywheelScale = 1.0 + kickerDroopFraction * flywheelFromKickerScale;
        double kickerScale = 1.0 + flywheelDroopFraction * kickerFromFlywheelScale;

        logCrossComp(baseFlywheelRPM * flywheelScale, baseKickerRPM * kickerScale,
                kickerDroopFraction, flywheelDroopFraction);
        SmartDashboard.putBoolean(CROSS_COMP_ACTIVE_KEY, flywheelFromKickerScale > 0.0 || kickerFromFlywheelScale > 0.0);
        SmartDashboard.putNumber(CROSS_COMP_APPLIED_TRANSIT_LEAD_SEC_KEY, transitLeadSec);
        SmartDashboard.putNumber(CROSS_COMP_PREDICTED_FLYWHEEL_RPM_KEY, predictedFlywheelRPM);
        SmartDashboard.putNumber(CROSS_COMP_PREDICTED_KICKER_RPM_KEY, predictedKickerRPM);

        return new CrossCompTargets(baseFlywheelRPM * flywheelScale, baseKickerRPM * kickerScale);
    }

    private static void logCrossComp(
            double commandedFlywheelRPM,
            double commandedKickerRPM,
            double kickerDroopFraction,
            double flywheelDroopFraction) {
        SmartDashboard.putNumber(COMMANDED_FLYWHEEL_RPM_KEY, commandedFlywheelRPM);
        SmartDashboard.putNumber(COMMANDED_KICKER_RPM_KEY, commandedKickerRPM);
        SmartDashboard.putNumber(KICKER_DROOP_FRACTION_LOG_KEY, kickerDroopFraction);
        SmartDashboard.putNumber(FLYWHEEL_DROOP_FRACTION_LOG_KEY, flywheelDroopFraction);
    }

    private void setShootState(ShootState newState) {
        if (m_shootState == newState) {
            return;
        }

        m_shootState = newState;
        m_stateStartTimeSec = Timer.getFPGATimestamp();
    }

    private double timeInState() {
        return Timer.getFPGATimestamp() - m_stateStartTimeSec;
    }

    private static double getKickerPullbackMinSec(ShotType effectiveShotType) {
        if (effectiveShotType == ShotType.PASS) {
            return Math.max(
                    0.0,
                    SmartDashboard.getNumber(
                            PASS_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC_DASHBOARD_KEY,
                            DEFAULT_PASS_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC));
        }

        return Math.max(
                0.0,
                SmartDashboard.getNumber(
                        NORMAL_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC_DASHBOARD_KEY,
                        DEFAULT_NORMAL_SHOT_KICKER_PULLBACK_HOLD_TIME_SEC));
    }

    private static boolean isPhasedControlEnabled() {
        return SmartDashboard.getBoolean(
                PHASED_SHOOT_CONTROL_ENABLED_DASHBOARD_KEY,
                DEFAULT_PHASED_SHOOT_CONTROL_ENABLED);
    }

    private static double getKickerSpinupFlywheelCompScale(ShotType effectiveShotType) {
        if (effectiveShotType == ShotType.PASS) {
            return Math.max(
                    1.0,
                    SmartDashboard.getNumber(
                            PASS_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP_DASHBOARD_KEY,
                            DEFAULT_PASS_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP));
        }

        return Math.max(
                1.0,
                SmartDashboard.getNumber(
                        NORMAL_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP_DASHBOARD_KEY,
                        DEFAULT_NORMAL_SHOT_FLYWHEEL_BOOST_WHILE_KICKER_SPINS_UP));
    }

    private static double getCrossCompLatencySec() {
        return Math.max(
                0.0,
                SmartDashboard.getNumber(
                        CROSS_COMP_SIGNAL_DELAY_COMP_SEC_DASHBOARD_KEY,
                        DEFAULT_CROSS_COMP_SIGNAL_DELAY_COMP_SEC));
    }

    private static double getCrossCompTransitLeadSec() {
        return MathUtil.clamp(
                SmartDashboard.getNumber(
                        CROSS_COMP_FUEL_TRAVEL_LOOKAHEAD_SEC_DASHBOARD_KEY,
                        DEFAULT_CROSS_COMP_FUEL_TRAVEL_LOOKAHEAD_SEC),
                0.0,
                0.20);
    }

    private static double getFinishAfterReleaseNoFuelSec() {
        return MathUtil.clamp(
                SmartDashboard.getNumber(
                        FINISH_AFTER_RELEASE_NO_FUEL_SEC_DASHBOARD_KEY,
                        DEFAULT_FINISH_AFTER_RELEASE_NO_FUEL_SEC),
                0.0,
                2.0);
    }

    private static boolean isFinishAfterReleaseEnabled() {
        return SmartDashboard.getBoolean(
                FINISH_AFTER_RELEASE_ENABLED_DASHBOARD_KEY,
                DEFAULT_FINISH_AFTER_RELEASE_ENABLED);
    }

    public static boolean isFinishAfterReleaseEnabledForBindings() {
        return isFinishAfterReleaseEnabled();
    }

    private static boolean isTurretOffTargetKickerPullbackEnabled() {
        return SmartDashboard.getBoolean(
                PULL_BACK_KICKER_WHEN_TURRET_OFF_TARGET_ENABLED_DASHBOARD_KEY,
                DEFAULT_PULL_BACK_KICKER_WHEN_TURRET_OFF_TARGET_ENABLED);
    }

    private boolean wantsShoot() {
        return m_wantsShootSupplier == null || m_wantsShootSupplier.getAsBoolean();
    }

    private static boolean isCrossCompEnabled(ShotType effectiveShotType) {
        if (effectiveShotType == ShotType.PASS) {
            return SmartDashboard.getBoolean(
                    PASS_SHOT_CROSS_COMP_ENABLED_DASHBOARD_KEY,
                    DEFAULT_PASS_SHOT_CROSS_COMP_ENABLED);
        }

        return SmartDashboard.getBoolean(
                NORMAL_SHOT_CROSS_COMP_ENABLED_DASHBOARD_KEY,
                DEFAULT_NORMAL_SHOT_CROSS_COMP_ENABLED);
    }

    private static boolean isFlywheelFromKickerCompEnabled(ShotType effectiveShotType) {
        if (effectiveShotType == ShotType.PASS) {
            return SmartDashboard.getBoolean(
                    PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED_DASHBOARD_KEY,
                    DEFAULT_PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED);
        }

        return SmartDashboard.getBoolean(
                NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED_DASHBOARD_KEY,
                DEFAULT_NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_ENABLED);
    }

    private static boolean isKickerFromFlywheelCompEnabled(ShotType effectiveShotType) {
        if (effectiveShotType == ShotType.PASS) {
            return SmartDashboard.getBoolean(
                    PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED_DASHBOARD_KEY,
                    DEFAULT_PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED);
        }

        return SmartDashboard.getBoolean(
                NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED_DASHBOARD_KEY,
                DEFAULT_NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_ENABLED);
    }

    private static double getFlywheelFromKickerScale(ShotType effectiveShotType) {
        if (effectiveShotType == ShotType.PASS) {
            return Math.max(
                    0.0,
                    SmartDashboard.getNumber(
                            PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE_DASHBOARD_KEY,
                            DEFAULT_PASS_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE));
        }

        return Math.max(
                0.0,
                SmartDashboard.getNumber(
                        NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE_DASHBOARD_KEY,
                        DEFAULT_NORMAL_SHOT_FLYWHEEL_HELP_FROM_KICKER_SCALE));
    }

    private static double getKickerFromFlywheelScale(ShotType effectiveShotType) {
        if (effectiveShotType == ShotType.PASS) {
            return Math.max(
                    0.0,
                    SmartDashboard.getNumber(
                            PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE_DASHBOARD_KEY,
                            DEFAULT_PASS_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE));
        }

        return Math.max(
                0.0,
                SmartDashboard.getNumber(
                        NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE_DASHBOARD_KEY,
                        DEFAULT_NORMAL_SHOT_KICKER_HELP_FROM_FLYWHEEL_SCALE));
    }

    private static double getFlywheelCrossCompMaxFraction() {
        return MathUtil.clamp(
                SmartDashboard.getNumber(
                        MAX_FLYWHEEL_CROSS_COMP_INCREASE_FRACTION_DASHBOARD_KEY,
                        DEFAULT_MAX_FLYWHEEL_CROSS_COMP_INCREASE_FRACTION),
                0.0,
                0.25);
    }

    private static double getKickerCrossCompMaxFraction() {
        return MathUtil.clamp(
                SmartDashboard.getNumber(
                        MAX_KICKER_CROSS_COMP_INCREASE_FRACTION_DASHBOARD_KEY,
                        DEFAULT_MAX_KICKER_CROSS_COMP_INCREASE_FRACTION),
                0.0,
                0.25);
    }

    private ShotSelection targetForShotType(Pose2d robotPose) {
        switch (m_shotType) {
            case HUB:
                return new ShotSelection(FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE), ShotType.HUB);
            case PASS:
                if (!FieldConstants.ENABLE_DYNAMIC_PASS_TARGETING) {
                    return new ShotSelection(FieldConstants.flipTranslation(standardPassTarget(selectionBlueLocation(robotPose))), ShotType.PASS);
                }
                return new ShotSelection(
                        FieldConstants.flipTranslation(calculatePassTarget(selectionBlueLocation(robotPose))),
                        ShotType.PASS);
            case OPPOSITE_ZONE:
                if (!FieldConstants.ENABLE_DYNAMIC_PASS_TARGETING) {
                    return new ShotSelection(FieldConstants.flipTranslation(standardPassTarget(selectionBlueLocation(robotPose))), ShotType.PASS);
                }
                return new ShotSelection(
                        FieldConstants.flipTranslation(calculatePassTarget(selectionBlueLocation(robotPose))),
                        ShotType.OPPOSITE_ZONE);
            // needed to suppress the warning
            // case TEST:
            //     return FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE);
            default:
                break;
        }
        return autoShotSelection(robotPose);
    }

    // Determine where we should shoot based on the robot location
    private ShotSelection autoShotSelection(Pose2d robotPose) {
        Translation2d blueLocation = selectionBlueLocation(robotPose);
        Translation2d target;
        ShotType shotType;
        if (FieldConstants.ENABLE_OPPOSITE_ZONE_SHOT
                && blueLocation.getX() >= FieldConstants.OPPOSITE_ALLIANCE_ZONE_START_X_BLUE) {
            m_latchedPassSide = null;
            target = oppositeAllianceZoneTarget(blueLocation);
            shotType = ShotType.OPPOSITE_ZONE;
        } else if (blueLocation.getX() < FieldConstants.HUB_POSITION_BLUE.getX()) {
            m_latchedPassSide = null;
            target = FieldConstants.HUB_POSITION_BLUE;
            shotType = ShotType.HUB;

        } else {
            target = FieldConstants.ENABLE_DYNAMIC_PASS_TARGETING
                    ? calculatePassTarget(blueLocation)
                    : standardPassTarget(blueLocation);
            shotType = ShotType.PASS;
        } 
        
        return new ShotSelection(FieldConstants.flipTranslation(target), shotType);
    }

    private Translation2d calculatePassTarget(Translation2d blueLocation) {
        double blueY = blueLocation.getY();
        boolean inCenterPassBand = blueY >= FieldConstants.PASSING_TARGET_LEFT_BLUE.getY()
                && blueY <= FieldConstants.PASSING_TARGET_RIGHT_BLUE.getY();

        if (!inCenterPassBand) {
            m_latchedPassSide = null;
            return new Translation2d(FieldConstants.PASSING_TARGET_LEFT_BLUE.getX(), blueY);
        }

        if (FieldConstants.ENABLE_PASS_SIDE_LATCH
                && m_latchedPassSide != null
                && passLatchStillValid(blueLocation, m_latchedPassSide)) {
            return passTargetForSide(m_latchedPassSide);
        }

        if (!FieldConstants.ENABLE_PASS_SIDE_LATCH) {
            return standardPassTarget(blueLocation);
        }

        if (m_latchedPassSide == null) {
            m_latchedPassSide = blueLocation.getY() < FieldConstants.FIELD_WIDTH / 2.0
                    ? PassSide.LEFT
                    : PassSide.RIGHT;
        }

        return m_latchedPassSide == PassSide.LEFT
                ? FieldConstants.PASSING_TARGET_LEFT_BLUE
                : FieldConstants.PASSING_TARGET_RIGHT_BLUE;
    }

    private static boolean passLatchStillValid(Translation2d blueLocation, PassSide latchedSide) {
        Translation2d latchedTarget = passTargetForSide(latchedSide);
        double distanceFromPassWall = Math.abs(blueLocation.getX() - latchedTarget.getX());
        double allowedYOffset = FieldConstants.PASS_LATCH_BASE_Y_TOLERANCE_BLUE
                + distanceFromPassWall * Math.tan(Math.toRadians(FieldConstants.PASS_LATCH_MAX_YAW_DEGREES));

        return Math.abs(blueLocation.getY() - latchedTarget.getY()) <= allowedYOffset;
    }

    private static Translation2d passTargetForSide(PassSide side) {
        return side == PassSide.LEFT
                ? FieldConstants.PASSING_TARGET_LEFT_BLUE
                : FieldConstants.PASSING_TARGET_RIGHT_BLUE;
    }

    private static Translation2d oppositeAllianceZoneTarget(Translation2d blueLocation) {
        boolean leftSide = blueLocation.getY() < FieldConstants.FIELD_WIDTH / 2.0;
        
        Translation2d nearTarget = leftSide
                ? FieldConstants.OPPOSITE_ZONE_TARGET_LINE_LEFT_NEAR_BLUE
                : FieldConstants.OPPOSITE_ZONE_TARGET_LINE_RIGHT_NEAR_BLUE;
        Translation2d farTarget = leftSide
                ? FieldConstants.OPPOSITE_ZONE_TARGET_LINE_LEFT_FAR_BLUE
                : FieldConstants.OPPOSITE_ZONE_TARGET_LINE_RIGHT_FAR_BLUE;

        double distanceFromSideWall = leftSide
                ? blueLocation.getY()
                : FieldConstants.FIELD_WIDTH - blueLocation.getY();
        double ratio = MathUtil.clamp(
                distanceFromSideWall / FieldConstants.SIDE_WALL_TARGET_LINE_MAX_DISTANCE_BLUE,
                0.0,
                1.0);

        return farTarget.interpolate(nearTarget, ratio);
    }

    public static Translation2d shotAutoTarget(Pose2d robotPose) {
        Translation2d blueLocation = selectionBlueLocation(robotPose);
        Translation2d target;

        if (FieldConstants.ENABLE_OPPOSITE_ZONE_SHOT
                && blueLocation.getX() >= FieldConstants.OPPOSITE_ALLIANCE_ZONE_START_X_BLUE) {
            target = oppositeAllianceZoneTarget(blueLocation);
        } else if (blueLocation.getX() < FieldConstants.HUB_POSITION_BLUE.getX()) {
            target = FieldConstants.HUB_POSITION_BLUE;
        } else if (!FieldConstants.ENABLE_DYNAMIC_PASS_TARGETING) {
            target = standardPassTarget(blueLocation);
        } else if (blueLocation.getY() >= FieldConstants.PASSING_TARGET_LEFT_BLUE.getY()
                && blueLocation.getY() <= FieldConstants.PASSING_TARGET_RIGHT_BLUE.getY()) {
            target = standardPassTarget(blueLocation);
        } else {
            target = new Translation2d(FieldConstants.PASSING_TARGET_LEFT_BLUE.getX(), blueLocation.getY());
        }

        return FieldConstants.flipTranslation(target);
    }

    public Translation2d findMovingShotVector(Pose2d currentPose, Translation2d target, ShotType effectiveShotType) {
        SmartDashboard.putString(EFFECTIVE_SHOT_TYPE_KEY, effectiveShotType.toString());
        SmartDashboard.putNumber(TARGET_X_KEY, target.getX());
        ChassisSpeeds speedInformation = m_speedsSupplier.get();
        Translation2d robotVelVector = new Translation2d(speedInformation.vxMetersPerSecond, speedInformation.vyMetersPerSecond);

        SmartDashboard.putNumber(ROBOT_VEL_KEY, robotVelVector.getNorm());
        SmartDashboard.putNumber(ROBOT_OMEGA_KEY, speedInformation.omegaRadiansPerSecond);

        Pose2d futureRobotPose = new Pose2d(
            currentPose.getTranslation().plus(robotVelVector.times(getMovingTranslationLatencySec())),
            currentPose.getRotation().plus(Rotation2d.fromRadians(
                    speedInformation.omegaRadiansPerSecond * getMovingRotationLatencySec()))
        );

        // Centripetal Velocity Calculator
        // This is the speed of the turret caused by the robot rotating
        double turretCentripetalSpeed = Math.abs(speedInformation.omegaRadiansPerSecond) * Turret.TURRET_OFFSET.getNorm();

        // net field direction of the "centripetal" velocity
        // do the sum directly to save some object constructors
        Rotation2d turretCentripetalDirection = Rotation2d.fromDegrees(
                futureRobotPose.getRotation().getDegrees() + 
                Turret.TURRET_OFFSET.getAngle().getDegrees() +
                Math.copySign(90.0, speedInformation.omegaRadiansPerSecond));
        
        Translation2d centripetalVelocity = new Translation2d(turretCentripetalSpeed, turretCentripetalDirection);
        // Translation2d centripetalVelocity = Translation2d.kZero;

        // net velocity of the turret: velocity of the robot's center, plus centripetal velocity around the center
        Translation2d turretVelTotal = robotVelVector.plus(centripetalVelocity);

        Pose2d lookaheadPose = futureRobotPose;

        Translation2d targetVector = Turret.getTranslationToGoal(lookaheadPose, target);
        double targetDistance = targetVector.getNorm();
        double previousTargetDistance = 0;
        double timeOfFlight = 0;

        for (int i = 0; i < 20; i++) {
            double lookupDistance = shotLookupDistance(lookaheadPose, targetVector, effectiveShotType);
            timeOfFlight = m_shooter.getShootValue(lookupDistance, effectiveShotType).timeOfFlight;
            // this is totally unsupported by real world!
            timeOfFlight *= getTofScale();
            Translation2d offset = turretVelTotal.times(timeOfFlight);

            lookaheadPose = new Pose2d(
                futureRobotPose.getTranslation().plus(offset),
                futureRobotPose.getRotation()
            );

            targetVector = Turret.getTranslationToGoal(lookaheadPose, target);
            targetDistance = targetVector.getNorm();

            if (Math.abs(targetDistance - previousTargetDistance) < 0.03) {
                // if the target distance did not change much, we've converged enough
                if (shouldPlotShotLocation()) {
                    m_turret.plotShotVectors(futureRobotPose, 
                            targetVector, robotVelVector.times(timeOfFlight),
                            centripetalVelocity.times(timeOfFlight));
                }                                   
                break;
            }

            previousTargetDistance = targetDistance;
        }

        if (shouldPlotShotLocation()) {
            m_turret.plotShotVectors(futureRobotPose,
                    targetVector, robotVelVector.times(timeOfFlight),
                    centripetalVelocity.times(timeOfFlight));
        }

        SmartDashboard.putNumber(SHOT_TOF_KEY, timeOfFlight);
        SmartDashboard.putNumber(TARGET_DISTANCE_KEY, targetDistance);

        return targetVector;
    }

    private static double shotLookupDistance(Pose2d robotPose, Translation2d shotVector, ShotType effectiveShotType) {
        return shotVector.getNorm();
    }

    private static Translation2d selectionBlueLocation(Pose2d robotPose) {
        if (FieldConstants.USE_TURRET_POSITION_FOR_SHOT_SELECTION) {
            Translation2d turret = blueTurretLocation(robotPose);
            double sign = 1.0;
            if (turret.getY() > FieldConstants.FIELD_WIDTH / 2.0)
                sign = -1.0;
            return turret.plus(new Translation2d(0.0, Units.inchesToMeters(sign*12.0)));
        }
        return FieldConstants.flipTranslation(robotPose.getTranslation());
    }

    private static Translation2d blueTurretLocation(Pose2d robotPose) {
        return FieldConstants.flipTranslation(Turret.getTurretFieldPosition(robotPose));
    }

    private static Translation2d standardPassTarget(Translation2d blueLocation) {
        return blueLocation.getY() < FieldConstants.FIELD_WIDTH / 2.0
                ? FieldConstants.PASSING_TARGET_LEFT_BLUE
                : FieldConstants.PASSING_TARGET_RIGHT_BLUE;
    }

    private ShootValue testShotValue() {
        return new ShootValue(
                SmartDashboard.getNumber(TEST_FLYWHEEL_RPM_KEY, 0.0),
                SmartDashboard.getNumber(TEST_KICKER_RPM_KEY, 0.0),
                Rotation2d.fromDegrees(SmartDashboard.getNumber(TEST_HOOD_ANGLE_KEY, 0.0)),
                SmartDashboard.getNumber(TEST_TIME_OF_FLIGHT_KEY, 0.0));
    }

    private static boolean shouldPlotShotLocation() {
        return SmartDashboard.getBoolean(PLOT_SHOT_LOCATION_KEY, RobotBase.isSimulation());
    }

    private static double getMovingTranslationLatencySec() {
        return Math.max(
                0.0,
                SmartDashboard.getNumber(
                        MOVING_SHOT_TRANSLATION_LOOKAHEAD_SEC_DASHBOARD_KEY,
                        DEFAULT_MOVING_SHOT_TRANSLATION_LOOKAHEAD_SEC));
    }

    private static double getMovingRotationLatencySec() {
        return Math.max(
                0.0,
                SmartDashboard.getNumber(
                        MOVING_SHOT_ROTATION_LOOKAHEAD_SEC_DASHBOARD_KEY,
                        DEFAULT_MOVING_SHOT_ROTATION_LOOKAHEAD_SEC));
    }

    private static double getTofScale() {
        return Math.max(
                0.0,
                SmartDashboard.getNumber(
                        MOVING_SHOT_TIME_OF_FLIGHT_SCALE_DASHBOARD_KEY,
                        DEFAULT_MOVING_SHOT_TIME_OF_FLIGHT_SCALE));
    }
}
