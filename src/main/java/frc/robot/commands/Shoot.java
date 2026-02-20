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
    private Translation2d m_target;

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
    @Override
    public void initialize() {
        switch (m_shotType) {
            case AUTO:
            case TEST:
                m_target = shotAutoTarget(m_poseSupplier.get());
                break;
            case HUB:
                m_target = FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE);
                break;
            case PASS:
                double yBlue = FieldConstants.flipTranslation(m_poseSupplier.get().getTranslation()).getY();
                if (yBlue < FieldConstants.FIELD_WIDTH / 2.0)
                    m_target = FieldConstants.flipTranslation(FieldConstants.PASSING_TARGET_LOWER_BLUE);
                else
                    m_target = FieldConstants.flipTranslation(FieldConstants.PASSING_TARGET_UPPER_BLUE);
                break;
                
            default:
                break;
        }
    }
    
    @Override
    public void execute() {
        Translation2d translationToHub = Turret.getTranslationToGoal(m_poseSupplier.get(), m_target);

        ShootValue shootValue;
        if (m_shotType == ShotType.TEST) {
            shootValue = testShootValue();
        } else {
            // Calculate distance and angle to target, send to shooter and turret subsystems
            shootValue = m_shooter.getShootValue(translationToHub.getNorm(), m_shotType);
        }
        
        // m_turret.setAngle(translationToHub.getAngle());
        m_shooter.setShootValues(shootValue);
        
        // Run feeder only when shooter and turret are ready
        if (m_shooter.onTarget()) {
            m_feeder.setRPM(shootValue.feedRPM);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
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

    private ShootValue testShootValue() {
        return new ShootValue(
                SmartDashboard.getNumber("flywheel/testRPM", 0.0),
                SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0),
                Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0)));
    }
}