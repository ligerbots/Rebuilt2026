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
import frc.robot.subsystems.shooter.Shooter.ShotType;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.utilities.ShooterLookupTable.ShootValue;

/**
* Command that coordinates shooting at the hub target.
* Manages turret aiming, shooter spin-up, and feeder activation.
*/
public class Shoot extends Command {
    static final boolean PLOT_SHOT_LOCATION = false;

    private final Shooter m_shooter;
    private final Turret m_turret;
    private final ShooterFeeder m_feeder;
    private final Hopper m_hopper;
    private final Supplier<ChassisSpeeds> m_speedsSupplier;
    private final Supplier<Pose2d> m_poseSupplier;

    private final Shooter.ShotType m_shotType;

    private static final double LATENCY_SECONDS_TRANSLATION = 0.05;
    private static final double LATENCY_SECONDS_ROTATION = 0.05;
    private static final double TOF_SCALE = 0.75;

    // for fixed shot only
    private final Translation2d m_fixedShotVector;

    // We want to "latch" the shooting on as soon as the flywheel is up
    //   to speed once. Otherwise, we turn off the feeder when the RPM drops - bad
    private boolean m_doShoot = false;

    private Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder, Hopper hopper,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, Shooter.ShotType shotType,
            double shotDistanceInches, Rotation2d turretHeading) {
        m_turret = turret;
        m_shooter = shooter;
        m_feeder = feeder;
        m_hopper = hopper;
        addRequirements(shooter, turret, feeder, hopper);

        m_poseSupplier = poseSupplier;
        m_speedsSupplier = speeds;

        m_shotType = shotType;

        m_fixedShotVector = new Translation2d(Units.inchesToMeters(shotDistanceInches), turretHeading);

        SmartDashboard.putNumber("hood/testAngle", 0.0);
        SmartDashboard.putNumber("flywheel/testRPM", 0.0);
        SmartDashboard.putNumber("kicker/testRPM", 0.0);
    }

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder, Hopper hopper,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, Shooter.ShotType shotType) {
        this(shooter, turret, feeder, hopper, poseSupplier, speeds, shotType, 0.0, Rotation2d.kZero);
    }

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder, Hopper hopper,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds,
            double shotDistanceInches, Rotation2d turretHeading) {
        this(shooter, turret, feeder, hopper, poseSupplier, speeds, ShotType.FIXED, shotDistanceInches, turretHeading);
    }

    @Override
    public void initialize() {
        m_doShoot = false;
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_poseSupplier.get();

        Translation2d shotVector;
        if (m_shotType == ShotType.FIXED) {
            shotVector = m_fixedShotVector;
            if (PLOT_SHOT_LOCATION) {
                m_turret.plotShotVectors(robotPose, shotVector, Translation2d.kZero, Translation2d.kZero);
            }
        } else {
            Translation2d target = targetForShotType();
            shotVector = findMovingShotVector(robotPose, target);
        }

        ShootValue shotValue;
        if (m_shotType == ShotType.TEST) {
            shotValue = testShotValue();
        } else {
            shotValue = m_shooter.getShootValue(shotVector.getNorm(), m_shotType);
        }

        Rotation2d angle = shotVector.getAngle();
        m_turret.setAngle(angle);
        m_shooter.setShootValues(shotValue);
        m_feeder.setKickerRPM(shotValue.feedRPM);

        SmartDashboard.putNumber("shoot/shotAngle", angle.getDegrees());

        if (!m_doShoot && m_shooter.onTarget() && m_turret.isOnTarget()) {
            m_doShoot = true;
        }

        if (m_doShoot && m_feeder.onTarget()) {
            m_hopper.feed();
            m_feeder.runFeederBelts();
            if (m_feeder.shouldRequestHopperPulse()) {
                m_hopper.requestPulse();
                m_feeder.onHopperPulseTriggered();
            }
        } else {
            m_hopper.reverse();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
        m_hopper.stop();

        if (PLOT_SHOT_LOCATION) {
            m_turret.plotShotVectors(null, null, null, null);
        }
    }

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
                if (yBlue < FieldConstants.FIELD_WIDTH / 2.0) {
                    return FieldConstants.flipTranslation(FieldConstants.PASSING_TARGET_LOWER_BLUE);
                }
                return FieldConstants.flipTranslation(FieldConstants.PASSING_TARGET_UPPER_BLUE);
            default:
                break;
        }
        return shotAutoTarget(m_poseSupplier.get());
    }

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

        SmartDashboard.putNumber("shoot/robotVel", robotVelVector.getNorm());
        SmartDashboard.putNumber("shoot/robotOmega", speedInformation.omegaRadiansPerSecond);

        Pose2d futureRobotPose = new Pose2d(
            currentPose.getTranslation().plus(robotVelVector.times(LATENCY_SECONDS_TRANSLATION)),
            currentPose.getRotation().plus(Rotation2d.fromRadians(speedInformation.omegaRadiansPerSecond * LATENCY_SECONDS_ROTATION))
        );

        double turretCentripetalSpeed = Math.abs(speedInformation.omegaRadiansPerSecond) * Turret.TURRET_OFFSET.getNorm();

        Rotation2d turretCentripetalDirection = Rotation2d.fromDegrees(
                futureRobotPose.getRotation().getDegrees()
                + Turret.TURRET_OFFSET.getAngle().getDegrees()
                + Math.copySign(90.0, speedInformation.omegaRadiansPerSecond));

        Translation2d centripetalVelocity = new Translation2d(turretCentripetalSpeed, turretCentripetalDirection);
        Translation2d turretVelTotal = robotVelVector.plus(centripetalVelocity);

        Pose2d lookaheadPose = futureRobotPose;

        Translation2d targetVector = Turret.getTranslationToGoal(lookaheadPose, target);
        double targetDistance = targetVector.getNorm();
        double previousTargetDistance = 0;
        double timeOfFlight = 0;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = m_shooter.getShootValue(targetDistance, m_shotType).timeOfFlight;
            timeOfFlight *= TOF_SCALE;
            Translation2d offset = turretVelTotal.times(timeOfFlight);

            lookaheadPose = new Pose2d(
                futureRobotPose.getTranslation().plus(offset),
                futureRobotPose.getRotation()
            );

            targetVector = Turret.getTranslationToGoal(lookaheadPose, target);
            targetDistance = targetVector.getNorm();

            if (Math.abs(targetDistance - previousTargetDistance) < 0.03) {
                if (PLOT_SHOT_LOCATION) {
                    m_turret.plotShotVectors(
                            futureRobotPose,
                            targetVector,
                            robotVelVector.times(timeOfFlight),
                            centripetalVelocity.times(timeOfFlight));
                }
                break;
            }

            previousTargetDistance = targetDistance;
        }

        SmartDashboard.putNumber("shoot/tof", timeOfFlight);
        SmartDashboard.putNumber("shoot/targetDistance", targetDistance);

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
