// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFeeder extends SubsystemBase {
    
    private static final double K_P = 0.2;
    private static final double K_D = 0.0;
    private static final double K_I = 0.0; 
    private static final double K_FF = 0.00217;  // V/rpm
    
    private static final double FEED_SUPPLY_CURRENT_LIMIT = 35;
    private static final double FEED_STATOR_CURRENT_LIMIT = 80;

    private static final double KICKER_SUPPLY_CURRENT_LIMIT = 30;
    private static final double KICKER_STATOR_CURRENT_LIMIT = 50;

    private static double FEEDER_BELT_FEED_VOLTAGE = 10.0;
    private static double FEEDER_BELT_UNJAM_VOLTAGE = -6.0;
    
    private final TalonFX m_motorKicker;
    private final TalonFX m_motorBelts;

    private double m_goalRPM;

    private final MotionMagicVelocityTorqueCurrentFOC m_velocityControl = new MotionMagicVelocityTorqueCurrentFOC(0);
    private final VoltageOut m_voltageControl = new VoltageOut(0).withEnableFOC(true);

    // Creates a new ShooterFeeder
    public ShooterFeeder() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  
        
        m_motorKicker = new TalonFX(Constants.SHOOTER_KICKER_CAN_ID);
        m_motorBelts = new TalonFX(Constants.SHOOTER_FEEDER_BELTS_CAN_ID);

        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;
        slot0configs.kI = K_I;
        slot0configs.kD = K_D;
        slot0configs.kV = K_FF * 60.0;   // K_FF is in V/rpm, motor uses rps

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(KICKER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(KICKER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        talonFXConfigs.withCurrentLimits(currentLimits);
        
        m_motorKicker.getConfigurator().apply(talonFXConfigs);
        // put kicker in Coast mode, so that it spins down slowly
        m_motorKicker.setNeutralMode(NeutralModeValue.Coast);

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(FEED_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(FEED_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        m_motorBelts.getConfigurator().apply(talonFXConfigs);
        // put feed belts in Brake mode so it stops quickly
        m_motorBelts.setNeutralMode(NeutralModeValue.Brake);


        if (Constants.OPTIMIZE_CAN) {
            optimizeCAN();
        }        
    }

    private void optimizeCAN() {
        m_motorKicker.getVelocity().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motorKicker.getMotorVoltage().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motorKicker.optimizeBusUtilization();

        m_motorBelts.getMotorVoltage().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motorBelts.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("kicker/currentRPM", getKickerRPM()); 
        SmartDashboard.putNumber("kicker/goalRPM", m_goalRPM);
        SmartDashboard.putNumber("kicker/statorCurrent", m_motorKicker.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("kicker/supplyCurrent", m_motorKicker.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("feeder/statorCurrent", m_motorBelts.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("feeder/supplyCurrent", m_motorBelts.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("feeder/voltage", m_motorBelts.getMotorVoltage().getValueAsDouble());
    }
    
    public double getKickerRPM(){
        return m_motorKicker.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
    }
    
    // public void setKickerVoltage(double rpm) {
    //     m_voltageControl.Output = rpm;
    //     m_motorKicker.setControl(m_voltageControl);
    // }

    public void setKickerRPM(double rpm) {
        m_goalRPM = rpm;
        m_velocityControl.Velocity = m_goalRPM / 60;   // velocity is in rot/second
        m_motorKicker.setControl(m_velocityControl);
    }
    
    public void setFeederBeltsVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motorBelts.setControl(m_voltageControl);
    }
    
    public void runFeederBelts() {
        setFeederBeltsVoltage(FEEDER_BELT_FEED_VOLTAGE);
    }

    public void runReverseUnjam() {
        setFeederBeltsVoltage(FEEDER_BELT_UNJAM_VOLTAGE);
    }

    public void stopFeederBelts() {
        m_motorBelts.set(0);
    }

    public void stop(){
        m_motorKicker.setVoltage(0);
        m_motorBelts.setVoltage(0);
        m_goalRPM = 0;
    }
}
