// TODO List:
// - Implement shuttle command (basically copy of this but with shuttle lookup table & changing shoot to target based off location)
// 
// 
// 




// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
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
  private static final Rotation2d TURRET_ANGLE_TOLERANCE = Rotation2d.fromDegrees(2.0); // TODO: Tune this value
  private static final double FEEDER_RPM_FOR_SHOOTING = 1500.0; // TODO: Tune this value

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
   * 
   * @TODO Integrate vision subsystem or odometry provider to get robot position
   *       for distance/angle calculations to the hub target
   */
  public ShootHub(Shooter shooter, Turret turret, ShooterFeeder feeder, CommandSwerveDrivetrain drivetrain) {
    addRequirements(shooter, turret, feeder);

    m_turret = turret;
    m_shooter = shooter;
    m_feeder = feeder;
    m_drivetrain = drivetrain;
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
    m_feeder.stop();
  }

  /**
   * Determines when the shoot command should end.
   * 
   * @return true when the command should terminate (currently stubbed)
   * 
   * TODO Define end condition: either when all game pieces are shot, when robot leaves shooting zone, or when match time expires
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getDistanceToTarget() {
    return getTurretPosition().getDistance(FieldConstants.HUB_POSITION);
  }

  private Rotation2d getAngleToTarget() {
    Rotation2d overallAngle = getTurretPosition().minus(FieldConstants.HUB_POSITION).getAngle();
    return overallAngle.plus(m_drivetrain.getState().Pose.getRotation());
  }

  private Translation2d getTurretPosition() {
    Pose2d robotPose = m_drivetrain.getState().Pose;
    return FieldConstants.TURRET_OFFSET.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
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