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
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  
  private final TalonFX m_motor;

  //need to calibrate the P value for the velocity loop, start small and increase until you get good response
  private static final double K_P = 3.0; 
  private static final int CURRENT_LIMIT = 90;

  //Trapisoidl:
  private static final double MAX_VEL_ROT_PER_SEC = 3; //TODO find new constants
  private static final double MAX_ACC_ROT_PER_SEC2 = 2;

  private double m_goalRPM;
  
  //Creates a new IntakeWheel
  public Flywheel() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  

    m_motor = new TalonFX(Constants.INTAKE_CAN_ID);
    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;  // start small!!!
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withStatorCurrentLimit(CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);

    MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        
    magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROT_PER_SEC;
    magicConfigs.MotionMagicAcceleration = MAX_ACC_ROT_PER_SEC2;

    // enable brake mode (after main config)
    m_motor.getConfigurator().apply(talonFXConfigs);
    m_motor.setNeutralMode(NeutralModeValue.Brake);

    MotorOutputConfigs m_motor = new MotorOutputConfigs();
    m_motor.Inverted = InvertedValue.Clockwise_Positive;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/currentRPM", getRPM()); 
    SmartDashboard.putNumber("Intake/goalRPM", m_goalRPM);
  }

  public double getRPM(){
    return m_motor.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
  }

  public void run(double rpm){
    m_goalRPM = rpm;

    // create a velocity closed-loop request, voltage output, slot 0 configs
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    double rps = m_goalRPM / 60; //convert rpm to rps

    // set velocity to 8 rps, add 0.5 V to overcome gravity
    m_motor.setControl(m_request.withVelocity(rps).withFeedForward(0.5));
  }

  public void stop(){
    m_motor.set(0);
  }
}
