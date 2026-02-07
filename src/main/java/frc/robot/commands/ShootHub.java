// TODO List:
// - Implement shuttle command (basically copy of this but with shuttle lookup table & changing shoot to target based off location)
// 
// 
// 




// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.Turret;

/**
 * Command that coordinates shooting at the hub target.
 * Manages turret aiming, shooter spin-up, and feeder activation.
 */
public class ShootHub extends Command {
  // Tolerance values for comparing actual vs target values
  private final Shooter m_shooter;
  private final Turret m_turret;
  private final ShooterFeeder m_feeder;
  private final CommandSwerveDrivetrain m_drivetrain;

  /**
   * Constructs a ShootHub command.
   * 
   * @param shooter The shooter subsystem for managing flywheel and hood
   * @param turret The turret subsystem for aiming at the hub
   * @param feeder The feeder subsystem for feeding game pieces
   * @param drivetrain
   */
  public ShootHub(Shooter shooter, Turret turret, ShooterFeeder feeder, CommandSwerveDrivetrain drivetrain) {
    m_turret = turret;
    m_shooter = shooter;
    m_feeder = feeder;
    m_drivetrain = drivetrain;

    addRequirements(shooter, turret, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShootType(Shooter.ShotType.HUB_SHOT);
  }

  @Override
  public void execute() {
    // Calculate distance and angle to target, send to shooter and turret subsystems
    double distanceToTarget = Turret.getDistanceToHub(m_drivetrain.getPose());
    m_shooter.setDistanceToTarget(distanceToTarget);

    Rotation2d angleToTarget = Turret.getAngleToHub(m_drivetrain.getPose());
    m_turret.setAngle(angleToTarget);

    // Run feeder only when shooter and turret are ready
    if (m_shooter.getCurrentState() == Shooter.ShooterState.READY_TO_SHOOT && m_turret.isTurretAtTargetForHub(m_drivetrain.getPose())) {
      m_feeder.feedForShooting();
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
   * 
   * TODO Define end condition: either when all game pieces are shot, when robot leaves shooting zone
   */
  @Override
  public boolean isFinished() {
    return false;
  }

}