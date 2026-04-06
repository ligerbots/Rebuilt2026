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

    private static record ShotSelection(Translation2d target, ShotType effectiveShotType) {}

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

    private static final double FLYWHEEL_SCALE = 0.96; //was 0.98 Q7

    // for fixed shot only
    private final Translation2d m_fixedShotVector;

    // Once we hit speed once, keep feeding even if RPM dips while shooting.
    private boolean m_shooterOnTarget = false;

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
            
            if (PLOT_SHOT_LOCATION) m_turret.plotShotVectors(robotPose, shotVector, Translation2d.kZero, Translation2d.kZero);
        } else {    
            ShotSelection shotSelection = targetForShotType();
            effectiveShotType = shotSelection.effectiveShotType();
            shotVector = findMovingShotVector(robotPose, shotSelection.target(), effectiveShotType);
            // old static shot
            // Translation2d translationToTarget = Turret.getTranslationToGoal(robotPose, target);
        }

        ShootValue shotValue;
        if (m_shotType == ShotType.TEST) {
            shotValue = testShotValue();
        } else {
            // Calculate distance and angle to target, send to shooter and turret subsystems
            shotValue = m_shooter.getShootValue(shotVector.getNorm(), effectiveShotType);
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

            if (PLOT_SHOT_LOCATION) m_turret.plotShotVectors(null, null, null, null);
        } else {
            // everything is good. Shoot!
            m_feeder.runFeederBelts();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();

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

    private ShotSelection targetForShotType() {
        switch (m_shotType) {
            case HUB:
                return new ShotSelection(FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE), ShotType.HUB);
            case PASS:
                double yBlue = FieldConstants.flipTranslation(m_poseSupplier.get().getTranslation()).getY();
                if (yBlue < FieldConstants.FIELD_WIDTH / 2.0)
                    return new ShotSelection(FieldConstants.flipTranslation(FieldConstants.PASSING_TARGET_LOWER_BLUE), ShotType.PASS);
                return new ShotSelection(FieldConstants.flipTranslation(FieldConstants.PASSING_TARGET_UPPER_BLUE), ShotType.PASS);
            // needed to suppress the warning
            // case TEST:
            //     return FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE);
            default:
                break;
        }
        return autoShotSelection(m_poseSupplier.get());
    }

    // Determine where we should shoot based on the robot location
    private static ShotSelection autoShotSelection(Pose2d robotPose) {
        Translation2d blueLocation = FieldConstants.flipTranslation(robotPose.getTranslation());
        Translation2d target;
        ShotType shotType;
        if (blueLocation.getX() < FieldConstants.HUB_POSITION_BLUE.getX()) {
            target = FieldConstants.HUB_POSITION_BLUE;
            shotType = ShotType.HUB;
        } else if (blueLocation.getY() < FieldConstants.FIELD_WIDTH / 2.0) {
            target = FieldConstants.PASSING_TARGET_LOWER_BLUE;
            shotType = ShotType.PASS;
        } else {
            target = FieldConstants.PASSING_TARGET_UPPER_BLUE;
            shotType = ShotType.PASS;
        }

        return new ShotSelection(FieldConstants.flipTranslation(target), shotType);
    }

    public static Translation2d shotAutoTarget(Pose2d robotPose) {
        return autoShotSelection(robotPose).target();
    }

    public Translation2d findMovingShotVector(Pose2d currentPose, Translation2d target, ShotType effectiveShotType) {
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
            timeOfFlight = m_shooter.getShootValue(targetDistance, effectiveShotType).timeOfFlight;
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
                if (PLOT_SHOT_LOCATION) {
                    m_turret.plotShotVectors(futureRobotPose, 
                            targetVector, robotVelVector.times(timeOfFlight),
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
