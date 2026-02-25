// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    // Tolerance values for comparing actual vs target values
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final ShooterFeeder m_feeder;
    private final Supplier<Pose2d> m_poseSupplier;

    private final Shooter.ShotType m_shotType;

    public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder, Supplier<Pose2d> poseSupplier, Shooter.ShotType shotType) {
        m_turret = turret;
        m_shooter = shooter;
        m_feeder = feeder;
        m_poseSupplier = poseSupplier;

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
        Pose2d robotPose = m_poseSupplier.get();
        Translation2d target = targetForShotType();
        Translation2d translationToTarget = Turret.getTranslationToGoal(robotPose, target);

        ShootValue shotValue;
        double shotDistance = 0;
        if (m_shotType == ShotType.TEST) {
            shotValue = testShotValue();
        } else {
            // Calculate distance and angle to target, send to shooter and turret subsystems
            shotDistance = translationToTarget.getNorm();
            shotValue = m_shooter.getShootValue(shotDistance, m_shotType);
        }
        
        m_turret.setAngle(translationToTarget.getAngle());
        m_shooter.setShootValues(shotValue);
        
        // Run feeder only when shooter and turret are ready
        if (m_shooter.onTarget()) {
            m_feeder.setRPM(shotValue.feedRPM);
        }
        m_turret.plotTurretHeading(robotPose, shotDistance);
}
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();

        // plot with 0 distance to turn it off
        m_turret.plotTurretHeading(null, 0);
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

    // Determines whether we should start shooting at the hub because we are in our zone.
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

    private ShootValue testShotValue() {
        return new ShootValue(
                SmartDashboard.getNumber("flywheel/testRPM", 0.0),
                SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0),
                Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0)));
    }
}