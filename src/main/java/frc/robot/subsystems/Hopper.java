// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    
    private final TalonFX m_motor;
    
    //need to calibrate the P value for the velocity loop, start small and increase until you get good response
    private static final double K_P = 3.0; 
    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = 60;
    
    private static final double K_FF = 0.0015; //TODO find new constant
    
    private static final double PULSE_VOLTAGE = 4.0;
    private static final double INTAKE_VOLTAGE = 2.0;

    private double m_goalRPM;
    
    //Creates a new IntakeWheel
    public Hopper() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  
        
        m_motor = new TalonFX(Constants.HOPPER_TRANSFER_CAN_ID);
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;  // start small!!!
        slot0configs.kI = 0.0;
        slot0configs.kD = 0.0;
        
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);
        
        // enable brake mode (after main config)
        m_motor.getConfigurator().apply(talonFXConfigs);
        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("hopper/currentRPM", getRPM()); 
        SmartDashboard.putNumber("hopper/goalRPM", m_goalRPM);
    }
    
    public void intake(){
        m_motor.setControl(new VoltageOut(INTAKE_VOLTAGE));
    }
    
    public void stop(){
        m_motor.setControl(new VoltageOut(0));
    }
    
    public double getRPM(){
        return m_motor.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
    }
    
    public void setVoltage(double volts) {
        m_motor.setControl(new VoltageOut(volts));
    }
    
    public void setRPM(double rpm){
        m_goalRPM = rpm;
        double rps = m_goalRPM / 60; //convert rpm to rps
        
        final VelocityVoltage m_request = new VelocityVoltage(rps).withFeedForward(K_FF * rpm);
        
        m_motor.setControl(m_request);
    }

    public Command pulseCommand() {
        return new InstantCommand(() -> setVoltage(PULSE_VOLTAGE))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(() -> setVoltage(0)))
            .andThen(new WaitCommand(0.05)).repeatedly().finallyDo(this::stop);
    }
}
