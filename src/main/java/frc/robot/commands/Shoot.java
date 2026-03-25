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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.FieldConstants;
import frc.robot.subsystems.Hopper;
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
    static final boolean PLOT_SHOT_LOCATION = false;

    // Tolerance values for comparing actual vs target values
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final ShooterFeeder m_feeder;
    private final Hopper m_hopper;
    private final Supplier<ChassisSpeeds> m_speedsSupplier;
    private final Supplier<Pose2d> m_poseSupplier;

    private final Shooter.ShotType m_shotType;

    private static final double LATENCY_SECONDS_TRANSLATION = 0.03;
    private static final double LATENCY_SECONDS_ROTATION = 0.05;

    // for fixed shot only
    private final Translation2d m_fixedShotVector;

    // We want to "latch" the shooting on as soon as the flywheel is up
    //   to speed once. Otherwise, we turn off the feeder when the RPM drops - bad
    private boolean m_flywheelOnTarget = false;

    private Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder, Hopper hopper,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, Shooter.ShotType shotType,
            double shotDistanceInches, Rotation2d turretHeading) {
        m_turret = turret;
        m_shooter = shooter;
        m_feeder = feeder;
        m_hopper = hopper;
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
    }

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder, Hopper hopper,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, Shooter.ShotType shotType) {
        this(shooter, turret, feeder, hopper,
                poseSupplier, speeds, shotType, 0.0, Rotation2d.kZero);
    }

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder, Hopper hopper,
                Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, 
                 double shotDistanceInches, Rotation2d turretHeading) {
        this(shooter, turret, feeder, hopper,
                poseSupplier, speeds, ShotType.FIXED, shotDistanceInches, turretHeading);
    }

    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // at start, wait for flywheel to get to speed
        m_flywheelOnTarget = false;
    }
    
    @Override
    public void execute() {
        // Pose is needed for plotting, so fetch it once here
        Pose2d robotPose = m_poseSupplier.get();

        Translation2d shotVector;
        if (m_shotType == ShotType.FIXED) {
            shotVector = m_fixedShotVector;
            if (PLOT_SHOT_LOCATION) m_turret.plotShotVectors(robotPose, shotVector, Translation2d.kZero, Translation2d.kZero);
        } else {    
            Translation2d target = targetForShotType();
            shotVector = findMovingShotVector(robotPose, target);
            // old static shot
            // Translation2d translationToTarget = Turret.getTranslationToGoal(robotPose, target);
        }

        ShootValue shotValue;
        if (m_shotType == ShotType.TEST) {
            shotValue = testShotValue();
        } else {
            // Calculate distance and angle to target, send to shooter and turret subsystems
            shotValue = m_shooter.getShootValue(shotVector.getNorm(), m_shotType);
        }
        
        m_turret.setAngle(shotVector.getAngle());
        m_shooter.setShootValues(shotValue);
        m_feeder.setKickerRPM(shotValue.feedRPM);
        
        // Once the flywheel is up to speed, latch it on.
        if (!m_flywheelOnTarget && m_shooter.onTarget() && m_turret.isOnTarget())
            m_flywheelOnTarget = true;

        // Run feeder only when shooter and turret are ready
        if (m_flywheelOnTarget) {
            m_feeder.runFeederBelts();
            m_hopper.feed();
        } else {
            m_feeder.runReverseUnjam();
        }
        // else if (!m_shooter.onTarget()) {
            // m_feeder.stopFeederBelts();
            // TODO: turret seemed to be interrupting the shot too much
            //  maybe widen the tolerance and re-enable this code?
            // m_feeder.stopFeederBelts();
            // m_hopper.stop();

            // if (PLOT_SHOT_LOCATION) m_turret.plotShotVectors(null, null, null, null);
        // }
    }
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
        m_hopper.stop();

        // erase our velocity vector scribblings
        if (PLOT_SHOT_LOCATION) m_turret.plotShotVectors(null, null, null, null);
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

    private Translation2d targetForShotType() {
        switch (m_shotType) {
            case HUB:
                return FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE);
            case PASS:
                double yBlue = FieldConstants.flipTranslation(m_poseSupplier.get().getTranslation()).getY();
                if (yBlue < FieldConstants.FIELD_WIDTH / 2.0)
                    return FieldConstants.flipTranslation(FieldConstants.PASSING_TARGET_LOWER_BLUE);
                return FieldConstants.flipTranslation(FieldConstants.PASSING_TARGET_UPPER_BLUE);
            // needed to suppress the warning
            // case TEST:
            //     return FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE);
            default:
                break;
        }
        return shotAutoTarget(m_poseSupplier.get());
    }

    // Determine where we should shoot based on the robot location
    public static Translation2d shotAutoTarget(Pose2d robotPose) {
        Translation2d blueLocation = FieldConstants.flipTranslation(robotPose.getTranslation());
        Translation2d target;
        if (blueLocation.getX() < FieldConstants.HUB_POSITION_BLUE.getX()) {
            target = FieldConstants.HUB_POSITION_BLUE;
        } else if (blueLocation.getY() < FieldConstants.FIELD_WIDTH / 2.0) {
            target = FieldConstants.PASSING_TARGET_LOWER_BLUE;
        } else {
            target = FieldConstants.PASSING_TARGET_UPPER_BLUE;
        }

        return FieldConstants.flipTranslation(target);
    }

    public Translation2d findMovingShotVector(Pose2d currentPose, Translation2d target) {
        ChassisSpeeds speedInformation = m_speedsSupplier.get();
        Translation2d robotVelVector = new Translation2d(speedInformation.vxMetersPerSecond, speedInformation.vyMetersPerSecond);

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
        // Translation2d turretVelTotal = robotVelVector.plus(centripetalVelocity);
        Translation2d turretVelTotal = robotVelVector;

        Pose2d lookaheadPose = futureRobotPose;

        Translation2d targetVector = Turret.getTranslationToGoal(lookaheadPose, target);
        double targetDistance = targetVector.getNorm();
        double previousTargetDistance = 0;

        for (int i = 0; i < 20; i++) {
            double timeOfFlight = m_shooter.getShootValue(targetDistance, m_shotType).timeOfFlight;
            Translation2d offset = turretVelTotal.times(timeOfFlight);

            lookaheadPose = new Pose2d(
                futureRobotPose.getTranslation().plus(offset),
                futureRobotPose.getRotation()
            );

            targetVector = Turret.getTranslationToGoal(lookaheadPose, target);
            targetDistance = targetVector.getNorm();

            if (Math.abs(targetDistance - previousTargetDistance) < 0.03) {
                // if the target distance did not change much, we've converged enough
                if (PLOT_SHOT_LOCATION) {
                    m_turret.plotShotVectors(futureRobotPose, 
                            targetVector, robotVelVector.times(timeOfFlight),
                            centripetalVelocity.times(timeOfFlight));
                }                                   
                break;
            }

            previousTargetDistance = targetDistance;
        }
       
        return targetVector;
    }

    private ShootValue testShotValue() {
        return new ShootValue(
                SmartDashboard.getNumber("flywheel/testRPM", 0.0),
                SmartDashboard.getNumber("kicker/testRPM", 0.0),
                Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0)),
                SmartDashboard.getNumber("shooter/testTimeOfFlight", 0.0));
    }
}