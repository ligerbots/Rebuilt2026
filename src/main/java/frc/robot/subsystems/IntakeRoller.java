// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class IntakeRoller extends SubsystemBase {
  
  private final TalonFX m_motor;

  //need to calibrate the P value for the velocity loop, start small and increase until you get good response
  private static final double K_P = 3.0; 
  private static final int CURRENT_LIMIT = 90;

  private static final double K_FF = 0.0015; //TODO find new constant

  private double m_goalRPM;
  
  //Creates a new IntakeWheel
  public IntakeRoller() {
    TalonFXConfiguration m_talonFXConfigs = new TalonFXConfiguration();  

    m_motor = new TalonFX(Constants.INTAKE_ROLLER_CAN_ID);
    Slot0Configs slot0configs = m_talonFXConfigs.Slot0;
    slot0configs.kP = K_P;  // start small!!!
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withStatorCurrentLimit(CURRENT_LIMIT);
    m_talonFXConfigs.withCurrentLimits(currentLimits);

    // enable brake mode (after main config)
    m_motor.getConfigurator().apply(m_talonFXConfigs);
    m_motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake/currentRPM", getRPM()); 
    SmartDashboard.putNumber("intake/goalRPM", m_goalRPM);
  }

  public double getRPM(){
    return m_motor.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
  }

  public void setRPM(double rpm){
    m_goalRPM = rpm;
    double rps = m_goalRPM / 60; //convert rpm to rps

    final VelocityVoltage m_request = new VelocityVoltage(rps).withFeedForward(K_FF * rpm);

    m_motor.setControl(m_request);
  }

  public void stop(){
    m_motor.set(0);
  }
}
