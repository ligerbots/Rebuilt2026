// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class ShooterFeeder extends SubsystemBase {
    
    //need to calibrate the P value for the velocity loop, start small and increase until you get good response
    private static final double K_P = 0.1; 
    private static final double K_FF = 0.0021; //TODO find new constant
    
    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = 60;
    
    private static final double FEEDER_RPM_FOR_SHOOTING = 1500.0; // TODO: Tune this value
    
    private double m_goalRPM;
    private final TalonFX m_motor;
        
    //Creates a new ShooterFeeder
    public ShooterFeeder() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  
        
        m_motor = new TalonFX(Constants.SHOOTER_FEEDER_CAN_ID);
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;  // start small!!!
        slot0configs.kI = 0.0;
        slot0configs.kD = 0.0;

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
        SmartDashboard.putNumber("shooterFeeder/currentRPM", getRPM()); 
        SmartDashboard.putNumber("shooterFeeder/goalRPM", m_goalRPM);
    }
    
    public double getRPM(){
        return m_motor.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
    }
    
    public void setVoltage(double voltage) {
        m_motor.setControl(new VoltageOut(voltage));
    }
    
    public void setRPM(double rpm){
        m_goalRPM = rpm;
        double rps = m_goalRPM / 60; //convert rpm to rps
        m_motor.setControl(new VelocityVoltage(rps).withFeedForward(K_FF * rpm));
    }
    
    public void stop(){
        m_motor.setVoltage(0);
    }
    
    public void feedForShooting() {
        setRPM(FEEDER_RPM_FOR_SHOOTING);
    }
}
