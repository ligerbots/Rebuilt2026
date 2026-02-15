// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.Turret;
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
  private final Supplier<Pose2d> m_drivetrainPoseSupplier;
  private final Shooter.ShotType m_shotType;

  /**
   * Constructs a ShootHub command.
   * 
   * @param shooter The shooter subsystem for managing flywheel and hood
   * @param turret The turret subsystem for aiming at the hub
   * @param feeder The feeder subsystem for feeding game pieces
   * @param drivetrain
   */
  public Shoot(Shooter shooter, Turret turret, ShooterFeeder feeder, Supplier<Pose2d> drivetrainPoseSupplier, Shooter.ShotType shotType) {
    m_turret = turret;
    m_shooter = shooter;
    m_feeder = feeder;
    m_drivetrainPoseSupplier = drivetrainPoseSupplier;
    m_shotType = shotType;

    addRequirements(shooter, turret, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShootType(m_shotType);
  }

  @Override
  public void execute() {
    // Translation2d goalTranslation;

    // if (m_shotType == Shooter.ShotType.HUB_SHOT) {
    //   goalTranslation = FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE);
    // } else {
    //   if (FieldConstants.FIELD_LENGTH/2.0 < m_drivetrainPoseSupplier.get().getX()) {
    //     goalTranslation = FieldConstants.mirrorTranslationX(FieldConstants.PASSING_TARGET_UPPER);
    //   } else {
    //     goalTranslation = FieldConstants.mirrorTranslationX(FieldConstants.PASSING_TARGET_LOWER);
    //   }
    // }

    Translation2d translationToHub = Turret.getTranslationToGoal(m_drivetrainPoseSupplier.get(), 
          FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE));

    // Calculate distance and angle to target, send to shooter and turret subsystems
    double distanceToTarget = translationToHub.getNorm();
    ShootValue shootValues = m_shooter.getShootValues(distanceToTarget, m_shotType);
    m_shooter.setShootValues(shootValues);

    Rotation2d angleToTarget = translationToHub.getAngle();
    m_turret.setAngle(angleToTarget);

    // Run feeder only when shooter and turret are ready
    if (m_shooter.onTarget()) {
      m_feeder.setRPM(shootValues.feedRPM);
      // m_hopper.run();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_feeder.stop();
    // m_hopper.stop();
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

}