// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRoller extends SubsystemBase {
    private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
    private static final Current STATOR_CURRENT_LIMIT = Amps.of(80);
    
    private static final double K_P = 0.01;
    private static final double K_I = 0.0; 
    private static final double K_D = 0.0;
    private static final double K_FF = 0.002135;  // V/rpm

    // private static final double INTAKE_VOLTAGE = 7.0;  // was 9
    // private static final double OUTTAKE_VOLTAGE = -6.0;

    private static final double INTAKE_RPM = 2750.0;
    private static final double FAST_INTAKE_RPM_SCALE = 1.5;
    private static final double OUTTAKE_RPM = -4000.0;

    private final TalonFX m_motor;

    private final VoltageOut m_voltageControl = new VoltageOut(0).withEnableFOC(true);
    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withEnableFOC(true);

    // private double m_intakeVoltageOffset = 0;
    // private static final double INTAKE_FUDGE = 0.5;
    private double m_intakeRPMScale = 1.0;
    private static final double INTAKE_RPM_FUDGE = 0.02;

    private double m_goalRPM;

    // Creates a new IntakeRoller
    public IntakeRoller() {
        m_motor = new TalonFX(Constants.INTAKE_ROLLER_CAN_ID);
        
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Config = talonFXConfigs.Slot0;
        slot0Config.kP = K_P;
        slot0Config.kI = K_I;
        slot0Config.kD = K_D;
        slot0Config.kV = K_FF * 60.0;   // K_FF is in V/rpm, motor uses rps

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        talonFXConfigs.withCurrentLimits(currentLimits);

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        m_motor.getConfigurator().apply(talonFXConfigs);
        // enable brake mode (after main config)
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
        SmartDashboard.putNumber("intake/RPM", getRPM()); 
        SmartDashboard.putNumber("intake/goalRPM", m_goalRPM); 
        SmartDashboard.putNumber("intake/voltageFudge", m_intakeRPMScale);
        SmartDashboard.putNumber("intake/rollerSupply", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("intake/rollerStator", m_motor.getStatorCurrent().getValueAsDouble());
    }
         
    public double getRPM(){
        return m_motor.getVelocity().getValueAsDouble() * 60; // convert rps to rpm
    }
    
    public void setRPM(double rpm) {
        m_goalRPM = rpm;
        m_velocityControl.Velocity = m_goalRPM / 60;   // velocity is in rot/second
        m_motor.setControl(m_velocityControl);
    }

    public void intake() {
        // setVoltage(INTAKE_VOLTAGE + m_intakeVoltageOffset);
        setRPM(INTAKE_RPM * m_intakeRPMScale);
    }

    public void fastIntake() {
        setRPM(INTAKE_RPM * FAST_INTAKE_RPM_SCALE * m_intakeRPMScale);
    }

    public void outtake() {
        // setVoltage(OUTTAKE_VOLTAGE);
        setRPM(OUTTAKE_RPM);
    }

    public void stop(){
        setVoltage(0);
        m_goalRPM = 0;
    }

    public void setVoltage(double volts) {
        m_voltageControl.Output = volts;
        m_motor.setControl(m_voltageControl);
    }

    public void increaseIntakeFudge() {
        m_intakeRPMScale += INTAKE_RPM_FUDGE;
    }
    public void decreaseIntakeFudge() {
        m_intakeRPMScale -= INTAKE_RPM_FUDGE;
    }
}
