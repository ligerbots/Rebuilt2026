// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    private enum HopperMode {
        STOPPED,
        FEEDING,
        REVERSING
    }
    
    private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(20);
    private static final Current STATOR_CURRENT_LIMIT =  Amps.of(70);

    private static final double PULSE_REVERSE_VOLTAGE = -6.5;
    private static final double PULSE_REVERSE_SEC = 0.05;

    private static final double INTAKE_VOLTAGE = 0.5;
    private static final double FEED_VOLTAGE = 10.0;
    private static final double REVERSE_VOLTAGE = -8.0;
    
    private final TalonFX m_motor;

    private final VoltageOut m_voltageControl = new VoltageOut(0);
    private final Timer m_pulseTimer = new Timer();
    private HopperMode m_mode = HopperMode.STOPPED;
    private boolean m_pulseActive = false;

    // Creates a new Hopper
    public Hopper() {
        m_motor = new TalonFX(Constants.HOPPER_TRANSFER_CAN_ID);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        talonFXConfigs.withCurrentLimits(currentLimits);
        
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // enable brake mode (after main config)
        m_motor.getConfigurator().apply(talonFXConfigs);
        m_motor.setNeutralMode(NeutralModeValue.Brake);

        if (Constants.OPTIMIZE_CAN) {
            optimizeCAN();
        }        
    }

    private void optimizeCAN() {
        m_motor.getMotorVoltage().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motor.optimizeBusUtilization();
    }
    
    @Override
    public void periodic() {
        if (m_pulseActive && m_pulseTimer.hasElapsed(PULSE_REVERSE_SEC)) {
            finishPulse();
        }

        SmartDashboard.putNumber("hopper/voltage", m_motor.getMotorVoltage().getValueAsDouble()); 
        SmartDashboard.putNumber("hopper/statorCurrent", m_motor.getStatorCurrent().getValueAsDouble()); 
        SmartDashboard.putNumber("hopper/supplyCurrent", m_motor.getSupplyCurrent().getValueAsDouble()); 
        SmartDashboard.putBoolean("hopper/pulseActive", isPulseActive());
    }
    
    public void intake(){
        m_mode = HopperMode.STOPPED;
        cancelPulse();
        setVoltage(INTAKE_VOLTAGE);
    }
    
    public void feed(){
        m_mode = HopperMode.FEEDING;
        if (!m_pulseActive) {
            setVoltage(FEED_VOLTAGE);
        }
    }
    

    public void reverse() {
        m_mode = HopperMode.REVERSING;
        cancelPulse();
        setVoltage(REVERSE_VOLTAGE);
    }

    public void stop(){
        m_mode = HopperMode.STOPPED;
        cancelPulse();
        setVoltage(0);
    }
    
    public double getRPM(){
        return m_motor.getVelocity().getValueAsDouble() * 60; // convert rps to rpm
    }
    
    public void setVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motor.setControl(m_voltageControl);
    }

    public void requestPulse() {
        if (isPulseActive()) {
            return;
        }

        m_pulseActive = true;
        m_pulseTimer.restart();
        setVoltage(PULSE_REVERSE_VOLTAGE);
    }

    public boolean isPulseActive() {
        return m_pulseActive;
    }

    private void finishPulse() {
        m_pulseActive = false;
        m_pulseTimer.stop();
        m_pulseTimer.reset();

        if (m_mode == HopperMode.FEEDING) {
            setVoltage(FEED_VOLTAGE);
        } else if (m_mode == HopperMode.REVERSING) {
            setVoltage(REVERSE_VOLTAGE);
        } else {
            setVoltage(0.0);
        }
    }

    private void cancelPulse() {
        m_pulseActive = false;
        m_pulseTimer.stop();
        m_pulseTimer.reset();
    }
}
