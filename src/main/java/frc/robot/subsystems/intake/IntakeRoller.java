// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class IntakeRoller extends SubsystemBase {
    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = 60;
    
    private static final double INTAKE_VOLTAGE = 9.0;  // was 9
    private static final double OUTTAKE_VOLTAGE = 3.0;

    private final TalonFX m_motor;
    private final VoltageOut m_voltageControl = new VoltageOut(0).withEnableFOC(true);

    // Creates a new IntakeRoller
    public IntakeRoller() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        m_motor = new TalonFX(Constants.INTAKE_ROLLER_CAN_ID);
        
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
        m_motor.getVelocity().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motor.getMotorVoltage().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/voltage", m_motor.getMotorVoltage().getValueAsDouble()); 
        SmartDashboard.putNumber("intake/RPM", m_motor.getVelocity().getValueAsDouble() * 60.0); 
    }
         
    public void intake() {
        setVoltage(INTAKE_VOLTAGE);
    }

    public void outtake() {
        setVoltage(OUTTAKE_VOLTAGE);
    }

    public void stop(){
        setVoltage(0);
    }

    public void setVoltage(double volts) {
        m_voltageControl.Output = volts;
        m_motor.setControl(m_voltageControl);
    }
}
