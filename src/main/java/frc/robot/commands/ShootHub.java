// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.Turret;

/**
 * Command that coordinates shooting at the hub target.
 * Manages turret aiming, shooter spin-up, and feeder activation.
 */
public class ShootHub extends Command {
  // Tolerance values for comparing actual vs target values
  private static final Rotation2d TURRET_ANGLE_TOLERANCE = Rotation2d.fromDegrees(2.0);
  private static final double FEEDER_RPM_FOR_SHOOTING = 1500.0; // TODO: Tune this value

  private final Shooter m_shooter;
  private final Turret m_turret;
  private final ShooterFeeder m_feeder;

  public ShootHub(Shooter shooter, Turret turret, ShooterFeeder feeder) {
    //TODO: Implement drivetrain to get current robot pose or require a supplier.
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, turret, feeder);

    m_turret = turret;
    m_shooter = shooter;
    m_feeder = feeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShootType(Shooter.ShootType.HUB_SHOT);
  }

  @Override
  public void execute() {
    // Calculate distance and angle to target, send to shooter and turret subsystems
    double distanceToTarget = getDistanceToTarget();
    m_shooter.setDistanceToTarget(distanceToTarget);

    Rotation2d angleToTarget = getAngleToTarget();
    m_turret.set(angleToTarget);

    // Run feeder only when shooter and turret are ready
    if (m_shooter.getCurrentState() == Shooter.ShooterState.READY_TO_SHOOT && isTurretAtTarget()) {
      m_feeder.setRPM(FEEDER_RPM_FOR_SHOOTING);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // TODO: Figure out when to end based on location & fuel state
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getDistanceToTarget() {
    return 0.0; // TODO: Implementation to get distance to target
  }

  private Rotation2d getAngleToTarget() {
    return Rotation2d.fromDegrees(0); // TODO: Implementation to get angle to target
  }

  /**
   * Checks if the turret is at the target angle within tolerance.
   * 
   * @return true if turret is within tolerance, false otherwise
   */
  private boolean isTurretAtTarget() {
    return Math.abs(m_turret.getPosition().minus(getAngleToTarget()).getRadians()) < TURRET_ANGLE_TOLERANCE.getRadians(); 
  }
}