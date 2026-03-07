// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    static final boolean PLOT_SHOT_LOCATION = true;

    // Tolerance values for comparing actual vs target values
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final ShooterFeeder m_feeder;
    private final Supplier<ChassisSpeeds> m_speedsSupplier;
    private final Supplier<Pose2d> m_poseSupplier;

    private final Shooter.ShotType m_shotType;

    private static final double LATENCY_SECONDS = 0.1; // TODO: tune to real value

    private static final double TRAVEL_TIME_SECONDS = 1.0; // TODO: tune to real value

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speeds, Shooter.ShotType shotType) {
        m_turret = turret;
        m_shooter = shooter;
        m_feeder = feeder;
        m_poseSupplier = poseSupplier;
        m_speedsSupplier = speeds;

        m_shotType = shotType;
        addRequirements(shooter, turret, feeder);

        // SD values used in the Test command
        SmartDashboard.putNumber("hood/testAngle", 0.0);
        SmartDashboard.putNumber("flywheel/testRPM", 0.0); 
        SmartDashboard.putNumber("shooterFeeder/testRPM", 0.0); 
    }
    
    // Called when the command is initially scheduled.
    // @Override
    // public void initialize() {
    // }
    
    @Override
    public void execute() {
        // Pose is needed for plotting, so fetch it once here
        Pose2d robotPose = m_poseSupplier.get();

        Translation2d target = targetForShotType();
        Translation2d shotVector = findMovingShotVector(robotPose, target);
        // Translation2d translationToTarget = Turret.getTranslationToGoal(robotPose, target);

        ShootValue shotValue;
        double shotDistance = shotVector.getNorm();
        if (m_shotType == ShotType.TEST) {
            shotValue = testShotValue();
        } else {
            // Calculate distance and angle to target, send to shooter and turret subsystems
            shotValue = m_shooter.getShootValue(shotVector.getNorm(), m_shotType);
        }
        
        m_turret.setAngle(shotVector.getAngle());
        m_shooter.setShootValues(shotValue);
        
        // Run feeder only when shooter and turret are ready
        if (m_shooter.onTarget() && m_turret.isOnTarget()) {
            m_feeder.setRPM(shotValue.feedRPM);
            if (PLOT_SHOT_LOCATION) m_turret.plotTurretHeading(robotPose, shotDistance);
        } else {
            m_feeder.stop();
            if (PLOT_SHOT_LOCATION) m_turret.plotTurretHeading(robotPose, 0);
        }
        
        // the turret is not simulated, so just update always
        if (PLOT_SHOT_LOCATION && RobotBase.isSimulation()) m_turret.plotTurretHeading(robotPose, shotDistance);
}
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();

        // plot with 0 distance to turn it off
        if (PLOT_SHOT_LOCATION) m_turret.plotTurretHeading(null, 0);
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
            currentPose.getTranslation().plus(robotVelVector.times(LATENCY_SECONDS)),
            currentPose.getRotation().plus(Rotation2d.fromRadians(speedInformation.omegaRadiansPerSecond * LATENCY_SECONDS))
        );

        // Centripetal Velocity Calculator
        // This is the speed of the turret caused by the robot rotating
        double turretCentripetalSpeed = speedInformation.omegaRadiansPerSecond * Turret.TURRET_OFFSET.getNorm();
        // net field direction of the "centripetal" velocity
        Rotation2d turretCentripetalDirection = futureRobotPose.getRotation().plus(Rotation2d.kCCW_90deg).plus(Turret.TURRET_OFFSET.getAngle());
        // centripetal velocity vector (velocity of turret around the center of the robot)
        Translation2d centripetalVelocity = new Translation2d(turretCentripetalSpeed, turretCentripetalDirection);

        // net velocity of the turret: velocity of the robot's center, plus centripetal velocity around the center
        Translation2d turretVelTotal = robotVelVector.plus(centripetalVelocity);

        Pose2d lookaheadPose = futureRobotPose;

        Translation2d targetVector = Turret.getTranslationToGoal(lookaheadPose, target);
        double previousTargetDistance = 0;

        for (int i = 0; i < 20; i++) {
            double targetDistance = targetVector.getNorm();
            double timeOfFlight = m_shooter.getShootValue(targetDistance, m_shotType).timeOfFlight;

            Translation2d offset = turretVelTotal.times(timeOfFlight);
            lookaheadPose = new Pose2d(
                futureRobotPose.getTranslation().plus(offset),
                futureRobotPose.getRotation()
            );

            targetVector = Turret.getTranslationToGoal(lookaheadPose, target);

            if (Math.abs(targetDistance - previousTargetDistance) < 0.03) {
                // if the target distance isn't changing much, we've converged enough
                break;
            }

            previousTargetDistance = targetDistance;
        }
       
        return targetVector;
    }

    private ShootValue testShotValue() {
        return new ShootValue(
                SmartDashboard.getNumber("flywheel/testRPM", 0.0),
                SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0),
                Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0)),
                SmartDashboard.getNumber("shooter/testTimeOfFlight", 0.0));

                
    }
}