// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
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
    private static final String PLOT_SHOT_LOCATION_KEY = "shoot/plotShotVisualization";

    private static record ShotSelection(Translation2d target, ShotType effectiveShotType) {}
    private enum PassSide {
        LEFT,
        RIGHT
    }

    private final Shooter m_shooter;
    private final Turret m_turret;
    private final ShooterFeeder m_feeder;
    private final Supplier<ChassisSpeeds> m_speedsSupplier;
    private final Supplier<Pose2d> m_poseSupplier;

    private final Shooter.ShotType m_shotType;

    private static final double LATENCY_SECONDS_TRANSLATION = 0.05;
    private static final double LATENCY_SECONDS_ROTATION = 0.05;

    // seem to need to scale the TOF numbers down
    private static final double TOF_SCALE = 0.75;

    private static final double FLYWHEEL_SCALE = 1.0;

    // for fixed shot only
    private final Translation2d m_fixedShotVector;

    // Once we hit speed once, keep feeding even if RPM dips while shooting.
    private boolean m_shooterOnTarget = false;
    private PassSide m_latchedPassSide = null;

    private Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, Shooter.ShotType shotType,
            double shotDistanceInches, Rotation2d turretHeading) {
        m_turret = turret;
        m_shooter = shooter;
        m_feeder = feeder;
        addRequirements(shooter, turret, feeder);
        
        m_poseSupplier = poseSupplier;
        m_speedsSupplier = speeds;

        m_shotType = shotType;

        // fixed shot only
        m_fixedShotVector = new Translation2d(Units.inchesToMeters(shotDistanceInches), turretHeading);

        // SD values used in the Test command
        SmartDashboard.putNumber("hood/testAngle", 0.0);
        SmartDashboard.putNumber("flywheel/testRPM", 0.0); 
        SmartDashboard.putNumber("kicker/testRPM", 0.0); 
        SmartDashboard.setDefaultBoolean(PLOT_SHOT_LOCATION_KEY, RobotBase.isSimulation());
    }

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, Shooter.ShotType shotType) {
        this(shooter, turret, feeder,
                poseSupplier, speeds, shotType, 0.0, Rotation2d.kZero);
    }

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder,
                Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, 
                 double shotDistanceInches, Rotation2d turretHeading) {
        this(shooter, turret, feeder,
                poseSupplier, speeds, ShotType.FIXED, shotDistanceInches, turretHeading);
    }

    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooterOnTarget = false;
        m_latchedPassSide = null;
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
        shotValue.flyRPM *= FLYWHEEL_SCALE;

        m_turret.setAngle(angle);
        m_shooter.setShootValues(shotValue);
        m_feeder.setKickerRPM(shotValue.feedRPM);
        
        SmartDashboard.putNumber("shoot/shotAngle", angle.getDegrees());

        if (!m_shooterOnTarget && m_shooter.onTarget()) {
            m_shooterOnTarget = true;
        }

        if (!m_shooterOnTarget || m_turret.inDeadZone()) {
            // if in the dead zone, turn off the feed
            m_feeder.stopFeederBelts();
        } else {
            // everything is good. Shoot!
            m_feeder.runFeederBelts();
        }

        if (!shouldPlotShotLocation()) {
            m_turret.clearShotVisualization();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();

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
        return false;
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
        SmartDashboard.putString("shoot/effectiveShotType", effectiveShotType.toString());
        SmartDashboard.putNumber("shoot/targetX", target.getX());
        ChassisSpeeds speedInformation = m_speedsSupplier.get();
        Translation2d robotVelVector = new Translation2d(speedInformation.vxMetersPerSecond, speedInformation.vyMetersPerSecond);

        SmartDashboard.putNumber("shoot/robotVel", robotVelVector.getNorm());
        SmartDashboard.putNumber("shoot/robotOmega", speedInformation.omegaRadiansPerSecond);

        Pose2d futureRobotPose = new Pose2d(
            currentPose.getTranslation().plus(robotVelVector.times(LATENCY_SECONDS_TRANSLATION)),
            currentPose.getRotation().plus(Rotation2d.fromRadians(speedInformation.omegaRadiansPerSecond * LATENCY_SECONDS_ROTATION))
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
            timeOfFlight *= TOF_SCALE;
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

        SmartDashboard.putNumber("shoot/tof", timeOfFlight);
        SmartDashboard.putNumber("shoot/targetDistance", targetDistance);

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
                SmartDashboard.getNumber("flywheel/testRPM", 0.0),
                SmartDashboard.getNumber("kicker/testRPM", 0.0),
                Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0)),
                SmartDashboard.getNumber("shooter/testTimeOfFlight", 0.0));
    }

    private static boolean shouldPlotShotLocation() {
        return SmartDashboard.getBoolean(PLOT_SHOT_LOCATION_KEY, RobotBase.isSimulation());
    }
}
