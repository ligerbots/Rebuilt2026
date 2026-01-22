// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class LinearExtension extends SubsystemBase {

  //TODO all values that are 0.123 are placeholders, set them properly

  private final TalonFX m_motor;

  private static final double DEPLOY_LENGTH = Units.inchesToMeters(0.123);
  private static final double STOW_LENGTH = Units.inchesToMeters(0.123);

  private static final double PINION_DIAMETER = Units.inchesToMeters(0.123);
  private static final double GEAR_REDUCTION = 0.123; //number of motor rotations for 1 pinion rotation

  private static final double K_P = 0.123;

  private static final double MAX_VEL_RAD_PER_SEC = 0.123;
  private static final double MAX_ACC_RAD_PER_SEC = 0.123;

  private static final double SUPPLY_CURRENT_LIMIT = 40;
  private static final double STATOR_CURRENT_LIMIT = 60;


  public LinearExtension() {
    
    m_motor = new TalonFX(0); //TODO use motor id constant

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;
    slot0configs.kI = 0.123;
    slot0configs.kD = 0.123;

    MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;

    magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_RAD_PER_SEC;
    magicConfigs.MotionMagicAcceleration = MAX_ACC_RAD_PER_SEC; //is this needed?

    m_motor.getConfigurator().apply(talonFXConfigs);
    m_motor.setPosition(0);

  }

  @Override
  public void periodic() {
 
    // This method will be called once per scheduler run
  }




  public void deploy() {

    double targetRotations = lengthToRotations(DEPLOY_LENGTH);
    m_motor.setControl(new MotionMagicVoltage(targetRotations));
    
    //rotate motor x times to make distance = DEPLOY_LENGTH
  }

  public void stow() {

    double targetRotations = lengthToRotations(STOW_LENGTH);
    m_motor.setControl(new MotionMagicVoltage(targetRotations));
    //retract
  }

  //maybe add a method that returns the current length extended?




  private double lengthToRotations(double length) {
    return (length / (Math.PI * PINION_DIAMETER)) * GEAR_REDUCTION;
  }


}
