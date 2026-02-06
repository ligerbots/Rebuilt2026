// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  
  private final TalonFX m_motor;

  //need to calibrate the P value for the velocity loop, start small and increase until you get good response
  private static final double K_P = 3.0;
  private static final double SUPPLY_CURRENT_LIMIT = 40;
  private static final double STATOR_CURRENT_LIMIT = 60;

  private static final double K_FF = 0.0015; //TODO find new constant


  private double m_goalRPM;
  
  //Creates a new FlyWheel
  public Flywheel() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  

    m_motor = new TalonFX(Constants.FLYWHEEL_CAN_ID);
    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;  // start small!!!
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);

    // enable coast mode (after main config)
    m_motor.getConfigurator().apply(talonFXConfigs);
    m_motor.setNeutralMode(NeutralModeValue.Coast);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("flywheel/currentRPM", getRPM()); 
    SmartDashboard.putNumber("flywheel/goalRPM", m_goalRPM);
  }

  public void setVoltage (double voltage) {
    m_motor.setControl(new VoltageOut(voltage));
  }

  public double getRPM(){
    return m_motor.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
  }

  public void setRPM(double rpm) {
    m_goalRPM = rpm;
    double rps = m_goalRPM / 60; //convert rpm to rps

    final VelocityVoltage m_request = new VelocityVoltage(rps).withFeedForward(K_FF * rpm);

    m_motor.setControl(m_request);
  }

  public void stop(){
    m_motor.setControl(new DutyCycleOut(0));
  }
}
