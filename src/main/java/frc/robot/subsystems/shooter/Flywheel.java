// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

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

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    private static final double SPEED_TOLERANCE_RPM = 175.0;

    private final TalonFX m_motor;
    private final TalonFX m_follower;

    private static final double K_P = 0.4;       // tuned 3/25
    private static final double K_D = 0.0;       // D>0 does not help 3/25
    private static final double K_FF = 0.0019;   // V / rpm

    private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
    private static final Current STATOR_CURRENT_LIMIT = Amps.of(70);

    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut m_voltageControl = new VoltageOut(0);

    private double m_goalRPM;

    public Flywheel() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        m_motor = new TalonFX(Constants.FLYWHEEL_CAN_ID);
        m_follower = new TalonFX(Constants.FLYWHEEL_FOLLOWER_CAN_ID);

        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;
        slot0configs.kI = 0.0;
        slot0configs.kD = K_D;
        slot0configs.kV = K_FF * 60.0;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        talonFXConfigs.withCurrentLimits(currentLimits);
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_motor.getConfigurator().apply(talonFXConfigs);
        m_motor.setNeutralMode(NeutralModeValue.Coast);

        m_follower.getConfigurator().apply(talonFXConfigs);
        m_follower.setNeutralMode(NeutralModeValue.Coast);
        m_follower.setControl(new Follower(m_motor.getDeviceID(), MotorAlignmentValue.Opposed));

        // DO NOT mess with the update frequency on the motors. This can affect
        // the leader/follower behavior. If we really need it, we will investigate.
        // https://www.chiefdelphi.com/t/setting-up-ctre-followers/512315/12
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("flywheel/currentRPM", getRPM());
        SmartDashboard.putNumber("flywheel/goalRPM", m_goalRPM);
        SmartDashboard.putNumber("flywheel/leaderStator", m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/followerStator", m_follower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/leaderSupply", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/followerSupply", m_follower.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/leaderTorqueCurrent", m_motor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/followerTorqueCurrent", m_follower.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/totalTorqueCurrent", getTotalTorqueCurrent());
    }

    public void setVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motor.setControl(m_voltageControl);
    }

    public double getRPM() {
        return m_motor.getVelocity().getValueAsDouble() * 60;
    }

    public double getTotalTorqueCurrent() {
        return m_motor.getTorqueCurrent().getValueAsDouble() + m_follower.getTorqueCurrent().getValueAsDouble();
    }

    public void setRPM(double rpm) {
        m_goalRPM = rpm;
        m_velocityControl.Velocity = m_goalRPM / 60;
        m_motor.setControl(m_velocityControl);
    }

    public boolean onTarget() {
        return Math.abs(getRPM() - m_goalRPM) < SPEED_TOLERANCE_RPM;
    }

    public void stop() {
        setVoltage(0);
        m_goalRPM = 0;
    }
}
