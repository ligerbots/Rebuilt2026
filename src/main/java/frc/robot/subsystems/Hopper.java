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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    
    private static final double SUPPLY_CURRENT_LIMIT = 20;
    private static final double STATOR_CURRENT_LIMIT =  70;

    private static final double PULSE_VOLTAGE = 4.0;
    private static final double INTAKE_VOLTAGE = 0.5;
    private static final double FEED_VOLTAGE = 3.0;
    private static final double REVERSE_VOLTAGE = -8.0;
    
    private final TalonFX m_motor;

    private final VoltageOut m_voltageControl = new VoltageOut(0);

    // Creates a new Hopper
    public Hopper() {
        m_motor = new TalonFX(Constants.HOPPER_TRANSFER_CAN_ID);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);
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
        SmartDashboard.putNumber("hopper/voltage", m_motor.getMotorVoltage().getValueAsDouble()); 
        SmartDashboard.putNumber("hopper/stator", m_motor.getStatorCurrent().getValueAsDouble()); 
        SmartDashboard.putNumber("hopper/supply", m_motor.getSupplyCurrent().getValueAsDouble()); 
    }
    
    public void intake(){
        setVoltage(INTAKE_VOLTAGE);
    }
    
    public void feed(){
        setVoltage(FEED_VOLTAGE);
    }
    

    public void reverse() {
        setVoltage(REVERSE_VOLTAGE);
    }

    public void stop(){
        setVoltage(0);
    }
    
    public double getRPM(){
        return m_motor.getVelocity().getValueAsDouble() * 60; // convert rps to rpm
    }
    
    public void setVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motor.setControl(m_voltageControl);
    }
    
    // public Command pulseCommand() {
    //     return new InstantCommand(() -> setVoltage(PULSE_VOLTAGE))
    //         .andThen(new WaitCommand(0.5))
    //         .andThen(new InstantCommand(() -> setVoltage(0)))
    //         .andThen(new WaitCommand(0.05)).repeatedly().finallyDo(this::stop);
    // }

    public Command pulseCommand() {
        return new InstantCommand(() -> setVoltage(8))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(() -> setVoltage(-2)))
            .andThen(new WaitCommand(0.05)).repeatedly().finallyDo(this::stop);
    }
}
