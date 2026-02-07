// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    private static final double SPEED_TOLERANCE_RPM = 100.0;    
    
    private final TalonFX m_motor;
    private final TalonFX m_follower;
    
    // need to calibrate the P value for the velocity loop, start small and increase until you get good response
    private static final double K_P = 0.1;   // TODO tune
    private static final double K_FF = 0.0021;

    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = 60;
        
    private double m_goalRPM;
    
    // Creates a new FlyWheel
    public Flywheel() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  
        
        m_motor = new TalonFX(Constants.FLYWHEEL_CAN_ID);
        m_follower = new TalonFX(Constants.FLYWHEEL_FOLLOWER_CAN_ID);
        
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;
        slot0configs.kI = 0.0;
        slot0configs.kD = 0.0;
        
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // enable coast mode (after main config)
        m_motor.getConfigurator().apply(talonFXConfigs);
        m_motor.setNeutralMode(NeutralModeValue.Coast);

        m_follower.getConfigurator().apply(talonFXConfigs);
        m_follower.setNeutralMode(NeutralModeValue.Coast);
        m_follower.setControl(new Follower(m_motor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("flywheel/currentRPM", getRPM()); 
        SmartDashboard.putNumber("flywheel/goalRPM", m_goalRPM);
        SmartDashboard.putNumber("flywheel/leaderCurrent", m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/followerCurrent", m_follower.getStatorCurrent().getValueAsDouble());
    }
    
    public void setVoltage(double voltage) {
        m_motor.setControl(new VoltageOut(voltage));
    }
    
    public double getRPM(){
        return m_motor.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
    }
    
    public void setRPM(double rpm) {
        m_goalRPM = rpm;
        double rps = m_goalRPM / 60; //convert rpm to rps
        
        m_motor.setControl(new VelocityVoltage(rps).withFeedForward(K_FF * rpm));
    }
    
    /**
    * Checks if the shooter is at the target RPM within tolerance.
    * 
    * @param targetRpm The target shooter RPM
    * @return true if shooter is within tolerance, false otherwise
    */
    public boolean onTarget() {
        return Math.abs(getRPM() - m_goalRPM) < SPEED_TOLERANCE_RPM;
    }
        
    public void stop(){
        setVoltage(0);
    }
}
