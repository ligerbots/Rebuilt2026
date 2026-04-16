// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.RobotLog;

public class ShooterFeeder extends SubsystemBase {
    
    private static final double KICKER_K_P = 0.6;
    private static final double KICKER_K_I = 0.0; 
    private static final double KICKER_K_D = 0.0;
    private static final double KICKER_K_FF = 0.00217;  // V/rpm

    private static final double KICKER_SUPPLY_CURRENT_LIMIT = 35;
    private static final double KICKER_STATOR_CURRENT_LIMIT = 90;

    private static final double FEEDER_K_P = 0.8;
    private static final double FEEDER_K_I = 0.0; 
    private static final double FEEDER_K_D = 0.0;
    private static final double FEEDER_K_FF = 0.0022;  // V/rpm
    
    private static final double FEEDER_SUPPLY_CURRENT_LIMIT = 35;
    private static final double FEEDER_STATOR_CURRENT_LIMIT = 90;

    // private static double FEEDER_BELT_FEED_VOLTAGE = 10.0;
    // private static double FEEDER_BELT_UNJAM_VOLTAGE = -6.0;

    private static double FEEDER_BELT_FEED_RPM = 4000.0;  // was 4250
    private static double FEEDER_BELT_UNJAM_RPM = -2700.0;

    private final TalonFX m_motorKicker;
    private final TalonFX m_motorFeeder;

    private double m_kickerGoalRPM;
    private double m_feederGoalRPM;

    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut m_voltageControl = new VoltageOut(0).withEnableFOC(true);

    // Creates a new ShooterFeeder
    public ShooterFeeder() {
        m_motorKicker = new TalonFX(Constants.SHOOTER_KICKER_CAN_ID);
        m_motorFeeder = new TalonFX(Constants.SHOOTER_FEEDER_BELTS_CAN_ID);

        TalonFXConfiguration kickerConfig = new TalonFXConfiguration();  
        Slot0Configs kickerSlot0Configs = kickerConfig.Slot0;
        kickerSlot0Configs.kP = KICKER_K_P;
        kickerSlot0Configs.kI = KICKER_K_I;
        kickerSlot0Configs.kD = KICKER_K_D;
        kickerSlot0Configs.kV = KICKER_K_FF * 60.0;   // K_FF is in V/rpm, motor uses rps
        kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs kickerCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(KICKER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(KICKER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        kickerConfig.withCurrentLimits(kickerCurrentLimits);
        
        m_motorKicker.getConfigurator().apply(kickerConfig);
        // put kicker in Coast mode, so that it spins down slowly
        m_motorKicker.setNeutralMode(NeutralModeValue.Coast);

        //-------

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        Slot0Configs beltsSlot0Configs = feederConfig.Slot0;
        beltsSlot0Configs.kP = FEEDER_K_P;
        beltsSlot0Configs.kI = FEEDER_K_I;
        beltsSlot0Configs.kD = FEEDER_K_D;
        beltsSlot0Configs.kV = FEEDER_K_FF * 60.0;   // K_FF is in V/rpm, motor uses rps

        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        CurrentLimitsConfigs beltsCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(FEEDER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(FEEDER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        feederConfig.withCurrentLimits(beltsCurrentLimits);

        m_motorFeeder.getConfigurator().apply(feederConfig);
        // put feed belts in Brake mode so they stop quickly
        m_motorFeeder.setNeutralMode(NeutralModeValue.Brake);

        if (Constants.OPTIMIZE_CAN) {
            optimizeCAN();
        }        
    }

    private void optimizeCAN() {
        m_motorKicker.getVelocity().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motorKicker.getMotorVoltage().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motorKicker.optimizeBusUtilization();

        m_motorFeeder.getVelocity().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motorFeeder.getMotorVoltage().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motorFeeder.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        // Driver-facing status
        SmartDashboard.putNumber("kicker/currentRPM", getKickerRPM()); 
        SmartDashboard.putNumber("feeder/currentRPM", getFeederRPM()); 
        
        // Kicker state
        RobotLog.log("kicker/goalRPM", m_kickerGoalRPM);
        RobotLog.log("kicker/statorCurrent", m_motorKicker.getStatorCurrent().getValueAsDouble());
        RobotLog.log("kicker/supplyCurrent", m_motorKicker.getSupplyCurrent().getValueAsDouble());

        // Feeder state
        RobotLog.log("feeder/goalRPM", m_feederGoalRPM);
        RobotLog.log("feeder/statorCurrent", m_motorFeeder.getStatorCurrent().getValueAsDouble());
        RobotLog.log("feeder/supplyCurrent", m_motorFeeder.getSupplyCurrent().getValueAsDouble());
        RobotLog.log("feeder/voltage", m_motorFeeder.getMotorVoltage().getValueAsDouble());
    }
    
    public double getKickerRPM(){
        return m_motorKicker.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
    }
    
    public void setKickerVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motorKicker.setControl(m_voltageControl);
    }

    public void setKickerRPM(double rpm) {
        m_kickerGoalRPM = rpm;
        m_velocityControl.Velocity = m_kickerGoalRPM / 60;   // velocity is in rot/second
        m_motorKicker.setControl(m_velocityControl);
    }
    
    public double getFeederRPM(){
        return m_motorFeeder.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
    }
    
    public void setFeederBeltsVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motorFeeder.setControl(m_voltageControl);
    }
    
    public void setFeederBeltsRPM(double rpm) {
        m_feederGoalRPM = rpm;
        m_velocityControl.Velocity = m_feederGoalRPM / 60;   // velocity is in rot/second
        m_motorFeeder.setControl(m_velocityControl);
    }

    public void runFeederBelts() {
        // setFeederBeltsVoltage(FEEDER_BELT_FEED_VOLTAGE);
        setFeederBeltsRPM(FEEDER_BELT_FEED_RPM);
    }

    public void runReverseUnjam() {
        // setFeederBeltsVoltage(FEEDER_BELT_UNJAM_VOLTAGE);
        setFeederBeltsRPM(FEEDER_BELT_UNJAM_RPM);
    }

    public void stopFeederBelts() {
        m_motorFeeder.set(0);
    }

    public void stop(){
        m_motorKicker.setVoltage(0);
        m_motorFeeder.setVoltage(0);
        m_kickerGoalRPM = 0;
        m_feederGoalRPM = 0;
    }
}
