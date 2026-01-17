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

  // define all constants here
  private final TalonFX m_motor;

  private static final double DEPLOY_LENGTH = 12; //TODO need to set number properly
  private static final double STOW_LENGTH = 0;

  private static final double GEAR_REDUCTION = 0;
  private static final double FINAL_GEAR_DIAMETER = 0; 
  private static final double METER_PER_REVOLUTION = 0;
  //these are filler numbers, set them properly later

  //TODO add in the lengths


  public LinearExtension() {

    m_motor = new TalonFX(); //add in motor id constant

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }




  public void deploy() {
    //extend
  }

  public void stow() {
    //retract
  }
}
