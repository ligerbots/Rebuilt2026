// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ShooterLookupTable;

public class Shooter extends SubsystemBase {

  private static final double SHOOTER_SPEED_TOLERANCE_RPM = 100.0;
  private static final Rotation2d HOOD_ANGLE_TOLERANCE = Rotation2d.fromDegrees(2.0);

  public enum ShooterState {
    IDLE,
    SPINNING_UP,
    READY_TO_SHOOT
  }

  public enum ShootType {
    HUB_SHOT,
    SHUTTLE_SHOT
  }

  private ShooterState m_currentState = ShooterState.IDLE;
  private ShootType m_shootType = ShootType.HUB_SHOT;


  private final ShooterLookupTable m_hubShooterLookupTable;
  private final ShooterLookupTable m_shuttleShooterLookupTable;

  private double m_targetDistanceMeters = 0.0;

  /**
   * Constructs a Shooter subsystem with lookup tables for hub and shuttle shooting.
   *
   * @param hubLookupTableFileName the file name for the lookup table for hub shooting calculations
   * @param shuttleLookupTableFileName the file name for the lookup table for shuttle shooting calculations
   */
  public Shooter(String hubLookupTableFileName, String shuttleLookupTableFileName) {
    m_hubShooterLookupTable = new ShooterLookupTable(hubLookupTableFileName);
    m_shuttleShooterLookupTable = new ShooterLookupTable(shuttleLookupTableFileName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    // If in IDLE state, do nothing (skip shooting logic below)
    if (m_currentState == ShooterState.IDLE) {
      return;
    }
    // Phase 1: Calculate distance to target and retrieve ballistic data

    ShooterLookupTable.ShootValue shootValues;
    if (m_shootType == ShootType.HUB_SHOT) {
      shootValues = m_hubShooterLookupTable.getShootValues(m_targetDistanceMeters);
    } else {
      shootValues = m_shuttleShooterLookupTable.getShootValues(m_targetDistanceMeters);
    }

    // Handle case where distance is out of range
    if (shootValues == null) {
      return;
    }

    // Phase 2: Set shooter speed, and hood angle
    setShooterSpeed(shootValues.m_rpm);
    setHoodAngle(shootValues.m_hoodAngle);

    // Phase 3: Check if all subsystems are at target values before running feeder
    if (isShooterAtTarget(shootValues.m_rpm) &&
      isHoodAtTarget(shootValues.m_hoodAngle)) {
        m_currentState = ShooterState.READY_TO_SHOOT;
    } else {
        m_currentState = ShooterState.SPINNING_UP;
    }
  }

  /**
   * Stops all shooter-related subsystems.
   */
  public void stop() {
    m_currentState = ShooterState.IDLE;
    setShooterSpeed(0.0);
    setHoodAngle(Rotation2d.fromDegrees(0));
  }



  /**
   * Sets the target distance for the shooter to use when calculating ballistic parameters.
   * 
   * @param distanceMeters The target distance in meters
   */
  public void setDistanceToTarget(double distanceMeters) {
    m_targetDistanceMeters = distanceMeters;
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
   * Gets the current state of the shooter.
   * 
   * @return The current ShooterState (IDLE, SPINNING_UP, or READY_TO_SHOOT)
   */
  public ShooterState getCurrentState() {
    return m_currentState;
  }
}
