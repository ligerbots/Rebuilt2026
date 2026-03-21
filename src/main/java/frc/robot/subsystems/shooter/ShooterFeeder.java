// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems.shooter;

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

public class ShooterFeeder extends SubsystemBase {
    
    private static final double K_P = 0.1; 
    private static final double K_FF = 0.0021;
    
    private static final double REVERSE_RPM = -1000.0;
    
    private static final double SUPPLY_CURRENT_LIMIT = 35;
    private static final double STATOR_CURRENT_LIMIT = 90;

    private static double FEEDER_BELT_FEED_VOLTAGE = 6.0; // TODO: tune this value
    private static double FEEDER_BELT_UNJAM_VOLTAGE = -6.0; // TODO: tune this value
    
    private double m_goalRPM;
    private final TalonFX m_motorKicker;
    private final TalonFX m_motorBelts;


    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut m_voltageControl = new VoltageOut(0);

    // Creates a new ShooterFeeder
    public ShooterFeeder() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  
        
        m_motorKicker = new TalonFX(Constants.SHOOTER_KICKER_CAN_ID);
        m_motorBelts = new TalonFX(Constants.SHOOTER_FEEDER_BELTS_CAN_ID);
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;
        slot0configs.kI = 0.0;
        slot0configs.kD = 0.0;

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);
        
        // enable brake mode (after main config)
        m_motorKicker.getConfigurator().apply(talonFXConfigs);
        m_motorKicker.setNeutralMode(NeutralModeValue.Coast);

        // TODO: figure if needs to be changed for 2nd motor
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_motorBelts.getConfigurator().apply(talonFXConfigs);
        m_motorBelts.setNeutralMode(NeutralModeValue.Brake);


        // if (Constants.OPTIMIZE_CAN) {
        //     optimizeCAN();
        // }        
    }

    private void optimizeCAN() {
        m_motorKicker.getVelocity().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motorKicker.getMotorVoltage().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motorKicker.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterFeeder/currentRPM", getKickerRPM()); 
        SmartDashboard.putNumber("shooterFeeder/goalRPM", m_goalRPM);
        SmartDashboard.putNumber("shooterFeeder/statorCurrent", m_motorKicker.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shooterFeeder/supplyCurrent", m_motorKicker.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shooterFeeder/belts/statorCurrent", m_motorBelts.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shooterFeeder/belts/supplyCurrent", m_motorBelts.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shooterFeeder/belts/voltage", m_motorBelts.getMotorVoltage().getValueAsDouble());
    }
    
    public double getKickerRPM(){
        return m_motorKicker.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
    }
    
    public void setKickerVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motorKicker.setControl(m_voltageControl);
    }

    public void setFeederBeltsVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motorBelts.setControl(m_voltageControl);
    }
    
    public void setKickerRPM(double rpm) {
        m_goalRPM = rpm;
        m_velocityControl.Velocity = m_goalRPM / 60;   // velocity is in rot/second
        m_velocityControl.FeedForward = K_FF * rpm;

        m_motorKicker.setControl(m_velocityControl);
    }
    
    public void runFeederBelts() {
        setFeederBeltsVoltage(FEEDER_BELT_FEED_VOLTAGE);
    }

    public void runReverseUnjam() {
        setFeederBeltsVoltage(FEEDER_BELT_UNJAM_VOLTAGE);
    }

    public void stopFeeder() {
        setKickerRPM(0); // TODO: add for belt feeder
    }

    public void stop(){
        m_motorKicker.setVoltage(0);
        m_motorBelts.setVoltage(0);
    }
}
