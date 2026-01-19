// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class ClimberArms extends SubsystemBase {

    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;

    // need to calibrate the P value for the velocity loop, start small and increase
    // until you get good response
    private static final double K_P = 3.0;
    private static final double K_FF = 0.0015; // TODO find new constant

    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = 60;

    private static final double MAX_VEL_RAD_PER_SEC = Units.degreesToRadians(50.0);
    private static final double MAX_ACC_RAD_PER_SEC = Units.degreesToRadians(50.0); // TODO change to better number (currently filler number)


    private double m_goalRPM;

    // Creates a new ClimberArms
    public ClimberArms() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        m_leftMotor = new TalonFX(Constants.CLIMBER_LEFT_MOTOR_CAN_ID);
        m_rightMotor = new TalonFX(Constants.CLIMBER_RIGHT_MOTOR_CAN_ID);
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P; // start small!!!
        slot0configs.kI = 0.0;
        slot0configs.kD = 0.0;

        MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        
        magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_RAD_PER_SEC;
        magicConfigs.MotionMagicAcceleration = MAX_ACC_RAD_PER_SEC;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);

        // enable brake mode (after main config)
        m_leftMotor.getConfigurator().apply(talonFXConfigs);
        m_leftMotor.setNeutralMode(NeutralModeValue.Brake);

        m_rightMotor.getConfigurator().apply(talonFXConfigs);
        m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterFeeder/currentRPM", getRPM());
        SmartDashboard.putNumber("shooterFeeder/goalRPM", m_goalRPM);
    }

    public double getRPM() {
        return m_motor.getVelocity().getValueAsDouble() * 60; // convert rps to rpm
    }

    public void setRPM(double rpm) {
        m_goalRPM = rpm;
        double rps = m_goalRPM / 60; // convert rpm to rps

        final VelocityVoltage m_request = new VelocityVoltage(rps).withFeedForward(K_FF * rpm);

        m_motor.setControl(m_request);
    }

    public void stop() {
        setRPM(0);
    }
}
