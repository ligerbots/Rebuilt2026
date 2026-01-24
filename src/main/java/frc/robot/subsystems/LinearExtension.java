// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class LinearExtension extends SubsystemBase {

  private final TalonFX m_motor;

  private static final double DEPLOY_LENGTH = Units.inchesToMeters(0.0); //TODO set properly
  private static final double STOW_LENGTH = Units.inchesToMeters(0.0); //TODO set properly

  private static final double PINION_DIAMETER = Units.inchesToMeters(0.0); //TODO set properly
  private static final double GEAR_REDUCTION = 0.0; //number of motor rotations for 1 pinion rotation

  private static final double ROTATIONS_TO_INCHES = GEAR_REDUCTION * Math.PI * PINION_DIAMETER;

  private static final double K_P = 0.0; //TODO set properly

  private static final double MAX_VEL_RAD_PER_SEC = 0.0; //TODO set properly
  private static final double MAX_ACC_RAD_PER_SEC = 0.0; //TODO set properly

  private static final double SUPPLY_CURRENT_LIMIT = 40;
  private static final double STATOR_CURRENT_LIMIT = 60;

  private double m_goal = 0;

  public LinearExtension() {
    
    m_motor = new TalonFX(Constants.LINEAR_EXTENSION_CAN_ID);

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

    talonFXConfigs.Feedback.withSensorToMechanismRatio(ROTATIONS_TO_INCHES);

    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;

    MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;

    magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_RAD_PER_SEC;
    magicConfigs.MotionMagicAcceleration = MAX_ACC_RAD_PER_SEC;

    m_motor.getConfigurator().apply(talonFXConfigs);
    m_motor.setPosition(0);

  }

  @Override
  public void periodic() {
 
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("linearExtension/currentLength", getLength());
    SmartDashboard.putNumber("linearExtension/goalLength", m_goal);
  }




  public void deploy() {
    setLength(DEPLOY_LENGTH);
    
  }

  public void stow() {
    setLength(STOW_LENGTH);
    //retract
  }


  public double getLength() {
    //returns current length extended
    return (m_motor.getPosition().getValueAsDouble());
    
  }

  public void setLength(double length) {
    m_motor.setControl(new MotionMagicVoltage(length));
    m_goal = length;
  }


}
