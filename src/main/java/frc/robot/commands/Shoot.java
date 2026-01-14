// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.ShootValues;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  // Tolerance values for comparing actual vs target values
  private static final Rotation2d TURRET_ANGLE_TOLERANCE = Rotation2d.fromDegrees(2.0);
  private static final double SHOOTER_SPEED_TOLERANCE_RPM = 100.0;
  private static final Rotation2d HOOD_ANGLE_TOLERANCE = Rotation2d.fromDegrees(2.0);

  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    // TODO: Add actual implementations to set turret angle, shooter speed, and hood angle
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    // Phase 1: Calculate distance to target and retrieve ballistic data
    double distanceToTarget = getDistanceToTarget();
    ShootValues shootValues = ShootValues.getShootValues(distanceToTarget);

    // Handle case where distance is out of range
    if (shootValues == null) {
      return;
    }

    // Phase 2: Set turret angle, shooter speed, and hood angle
    Rotation2d angleToTarget = getAngleToTarget();
    setTurretAngle(angleToTarget);
    setShooterSpeed(shootValues.m_rpm);
    setHoodAngle(shootValues.m_hoodAngle);

    // Phase 3: Check if all subsystems are at target values before running feeder
    if (isTurretAtTarget(angleToTarget) &&
        isShooterAtTarget(shootValues.m_rpm) &&
        isHoodAtTarget(shootValues.m_hoodAngle)) {
      runFeeder();
    }
  }

  /**
   * Called once the command ends or is interrupted.
   * Stops all shooter subsystems.
   */
  @Override
  public void end(boolean interrupted) {
    // Stop all shooter-related subsystems
    setShooterSpeed(0.0);
    stopFeeder(); // This should have a stop implementation
  }

  // Returns true when the command should end.
  // TODO: Figure out when to end being location & fuel dependent
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Gets the current distance from the robot to the target.
   * 
   * @return Distance in meters
   */
  private double getDistanceToTarget() {
    // TODO: Implementation to get distance to target
    return 0.0;
  }

  /**
   * Gets the current angle from the robot to the target.
   * 
   * @return Angle as a Rotation2d object
   */
  private Rotation2d getAngleToTarget() {
    // TODO: Implementation to get angle to target
    // This should calculate the angle needed to point the turret at the target
    return Rotation2d.fromDegrees(0.0);
  }

  /**
   * Sets the turret to a specific angle.
   * 
   * @param angle The target angle as a Rotation2d object
   */
  private void setTurretAngle(Rotation2d angle) {
    // TODO: Implementation to set the turret angle via turret subsystem
    // This should command the turret motor to rotate to the target angle
  }

  /**
   * Sets the shooter motor to a specific RPM.
   * 
   * @param speed The target RPM for the shooter motor
   */
  private void setShooterSpeed(double speed) {
    // TODO: Implementation to set the shooter speed via shooter subsystem
    // This should command the shooter motor to spin at the target RPM
  }

  /**
   * Sets the hood to a specific angle.
   * 
   * @param angle The target angle as a Rotation2d object
   */
  private void setHoodAngle(Rotation2d angle) {
    // TODO: Implementation to set the hood angle via hood subsystem
  }

  /**
   * Checks if the turret is at the target angle within tolerance.
   * 
   * @param targetAngle The target turret angle as a Rotation2d object
   * @return true if turret is within tolerance, false otherwise
   */
  private boolean isTurretAtTarget(Rotation2d targetAngle) {
    // TODO: Get current turret angle from turret subsystem and compare
    // return currentTurretAngle.minus(targetAngle).abs().getRadians() < TURRET_ANGLE_TOLERANCE.getRadians();
    return false;
  }

  /**
   * Checks if the shooter is at the target RPM within tolerance.
   * 
   * @param targetRpm The target shooter RPM
   * @return true if shooter is within tolerance, false otherwise
   */
  private boolean isShooterAtTarget(double targetRpm) {
    // TODO: Get current shooter RPM from shooter subsystem and compare
    // return Math.abs(currentShooterRpm - targetRpm) < SHOOTER_SPEED_TOLERANCE_RPM;
    return false;
  }

  /**
   * Checks if the hood is at the target angle within tolerance.
   * 
   * @param targetAngle The target hood angle as a Rotation2d object
   * @return true if hood is within tolerance, false otherwise
   */
  private boolean isHoodAtTarget(Rotation2d targetAngle) {
    // TODO: Get current hood angle from hood subsystem and compare
    // return currentHoodAngle.minus(targetAngle).abs().getRadians() < HOOD_ANGLE_TOLERANCE.getRadians();
    return false;
  }

  /**
   * Runs the feeder to shoot the ball.
   */
  private void runFeeder() {
    // TODO: Implementation to run the feeder via feeder subsystem
    // This should command the feeder motor to push the ball into the shooter
  }

  private void stopFeeder() {
    // TODO: Implementation to run the feeder via feeder subsystem
    // This should command the feeder motor to push the ball into the shooter
  }
}