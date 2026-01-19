// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
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

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * Executes the three-phase shooting logic:
   * Phase 1: Get distance and angle to target, retrieve shoot values from lookup table
   * Phase 2: Set turret angle, shooter speed, and hood angle
   * Phase 3: If all values are within tolerance, run the feeder
   */
  @Override
  public void execute() {
    // Phase 1: Calculate distance to target and send to shooter subsystem
    double distanceToTarget = getDistanceToTarget();
    m_shooter.setDistanceToTarget(distanceToTarget);

    Rotation2d angleToTarget = getAngleToTarget();
    m_turret.set(angleToTarget);

    // Phase 3: Check if all subsystems are at target values before running feeder
    if (m_shooter.getCurrentState() == Shooter.ShooterState.READY_TO_SHOOT && isTurretAtTarget()) {
      m_feeder.setRPM(FEEDER_RPM_FOR_SHOOTING);
    }
  }

  /**
   * Called once the command ends or is interrupted.
   * Stops all shooter subsystems.
   */
  @Override
  public void end(boolean interrupted) {
    // Stop all shooter-related subsystems
    m_shooter.stop();
  }

  // Returns true when the command should end.
  // TODO: Figure out when to end being location & fuel dependent
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