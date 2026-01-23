// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//look for motor ratios
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Turret extends SubsystemBase {

  private Rotation2d m_goal = Rotation2d.fromDegrees(0.0);

  private final TalonFX m_turretMotor;

  private static final double SUPPLY_CURRENT_LIMIT = 60;
  private static final double STATOR_CURRENT_LIMIT = 40;

  // TODO Tune the constants
  private static final double K_P = 1.0; 
  private static final double MAX_VEL_ROT_PER_SEC = 1;  // Target cruise velocity of 80 rps
  private static final double MAX_ACC_ROT_PER_SEC_SQ = 0.5;   // Target acceleration of 160 rps/s (0.5 seconds) 
  private static final Rotation2d MAX_ROTATION = Rotation2d.fromRadians(2 * Math.PI);
  private static final Rotation2d MIN_ROTATION = Rotation2d.fromRadians(0.0);

  private static final double ROTATIONS_PER_INCHES = 3; // TODO change to how many real rotation does it takes to extend 1 inch


  /** Creates a new Turret. */
  public Turret() {
    m_turretMotor = new TalonFX(0); //TODO add the motor id constant
  
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    
    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;

    MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
    magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROT_PER_SEC; 
    magicConfigs.MotionMagicAcceleration = MAX_ACC_ROT_PER_SEC_SQ; 

    talonFXConfigs.Feedback.withSensorToMechanismRatio(ROTATIONS_PER_INCHES);

    m_turretMotor.getConfigurator().apply(talonFXConfigs);
    m_turretMotor.setPosition(0);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {    
    SmartDashboard.putNumber("turret/goalAngle", m_goal.getRadians());
    SmartDashboard.putNumber("turret/currentAngle", getPosition().getRadians());
    SmartDashboard.putNumber("turret/rawMotorAngle", m_turretMotor.getPosition().getValueAsDouble());
  }

  public void set(Rotation2d angle) {
    m_goal = limitRotation(angle);
    m_turretMotor.setControl(new MotionMagicVoltage(m_goal.getRotations()));
  }

  public Rotation2d getPosition(){
    return Rotation2d.fromRotations(m_turretMotor.getPosition().getValueAsDouble());
  }

  // max rotation and min rotation in constants
  // mathutil.clamp
  private Rotation2d limitRotation(Rotation2d angle) {
    return Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), MIN_ROTATION.getRadians(), MAX_ROTATION.getRadians()));
  }
}