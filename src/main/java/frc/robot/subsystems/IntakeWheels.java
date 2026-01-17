// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWheels extends SubsystemBase {
  public enum IntakeState {
    STOW, DEPLOY
  }

  private static final double motorPower = 0;
  private IntakeState m_state = IntakeState.STOW;

  private final Rotation2d STOWED_ANGLE = Rotation2d.fromDegrees(0);
  private final Rotation2d DEPLOYED_ANGLE = Rotation2d.fromDegrees(0); 

  private final double ROLLER_INTAKE_SPEED = 0;
  private final double ROLLER_OUTTAKE_SPEED = 0; 
  private final double ROLLER_HOLD_SPEED = 0; 

  /**Functions: Spin at a given motor power % (Possibly RPM if we desire, don’t implement though)
  Sensors: Internal encoders
  Motors: Kraken X44
  Smart Dashboard output: RPM, Goal RPM*/ 
  /** Creates a new intakeWheels. */
  public IntakeWheels() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (m_state) {
      case STOW:
        setRollerSpeedPercent(0);
        setAngleWithProfile(STOWED_ANGLE);
        break;
      case DEPLOY:
        setRollerSpeedPercent(ROLLER_INTAKE_SPEED);
        setAngleWithProfile(DEPLOYED_ANGLE);
        goToScoreAngle();
        break;
    }
  }
  private void setAngleWithProfile(Rotation2d angle) {
    m_goalAngle = angle.getDegrees();

    State goalState = new State(angle.getRotations(), 0);

    // Trapezoid Profile
    m_currentState = m_profile.calculate(ROBOT_LOOP_PERIOD, m_currentState, goalState);
    m_pivotController.setReference(m_currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);

    public Rotation2d getRollerSpeed() {
      return Rotation2d.fromRotations(m_rollerMotor.getEncoder().getVelocity());
    }

    // Get the current pivot angle
    public Rotation2d getPivotAngle() {
      return Rotation2d.fromRotations(m_pivotMotor.getEncoder().getPosition());
    }
    public void setPivotAngle(Rotation2d angle, double feedForward) {
      m_goalAngle = angle.getDegrees();
      m_pivotController.setReference(angle.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
    }
    // set the speed of the roller in RPM
    public void setRollerSpeedPercent(double speed) {
      m_rollerMotor.set(speed);
    }

    public void stow() {
      m_state = IntakeState.STOW;
    }
    public void deploy() {
      m_state = IntakeState.DEPLOY;
    }
    public void goToScoreAngle() {
      m_state = IntakeState.SCORE_ANGLE;
    }
    public IntakeState getState() {
      return m_state;
    }
  }
}
