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

  private final TalonFX m_motor;

  private static final double DEPLOY_LENGTH = 12; //TODO need to set number properly
  private static final double STOW_LENGTH = 0;

  private static final double PINION_DIAMETER = 0;
  private static final double MOTOR_ROTATIONS_PER_PINION_ROTATION = 0; //TODO rename this
  private static final double PINION_ROTATIONS_PER_INCH = 0; //TODO rename this

  //private static final double GEAR_REDUCTION = 0;
  //private static final double FINAL_GEAR_DIAMETER = 0; 
  //private static final double METER_PER_REVOLUTION = 0;

  private static final double K_P = 1.0; //TODO edit this val??

  private static final double MAX_VEL_RAD_PER_SEC = 0;
  private static final double MAX_ACC_RAD_PER_SEC = 0; //TODO edit these vals

  private static final double SUPPLY_CURRENT_LIMIT = 40;
  private static final double STATOR_CURRENT_LIMIT = 60; //TODO set the right nums



  //these are filler numbers, set them properly later

  //TODO add in the lengths


  public LinearExtension() {
    
    m_motor = new TalonFX(0); //TODO use motor id constant

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0; //TODO edit these vals

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
    


   
    //rotate motor x times to make distance = DEPLOY_LENGTH
  }

  public void stow() {
    //retract
  }


}
