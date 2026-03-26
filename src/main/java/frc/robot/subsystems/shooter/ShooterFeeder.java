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

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFeeder extends SubsystemBase {
    private static final double SPEED_TOLERANCE_RPM = 100.0;
    private static final int PULSE_FILTER_TAPS = 5;
    private static final int PULSE_DEBOUNCE_CYCLES = 40;
    private static final int PULSE_COOLDOWN_CYCLES = 30;

    private static final double BELTS_HIGH_STATOR_CURRENT_AMPS = 55.0;
    private static final double BELTS_LOW_RPM_THRESHOLD = 3000.0;

    private static final double K_P = 0.2;
    private static final double K_D = 0.0;
    private static final double K_I = 0.0;
    private static final double K_FF = 0.00217; // V/rpm

    private static final double KICKER_SUPPLY_CURRENT_LIMIT = 35.0;
    private static final double KICKER_STATOR_CURRENT_LIMIT = 90.0;
    private static final double BELTS_SUPPLY_CURRENT_LIMIT = 30.0;
    private static final double BELTS_STATOR_CURRENT_LIMIT = 90.0;

    private static final double FEEDER_BELT_FEED_VOLTAGE = 11.0;
    private static final double FEEDER_BELT_UNJAM_VOLTAGE = -6.0;

    private final TalonFX m_motorKicker;
    private final TalonFX m_motorBelts;

    private double m_goalRPM;

    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut m_beltsVoltageControl = new VoltageOut(0).withEnableFOC(true);

    private final LinearFilter m_beltsRpmFilter = LinearFilter.movingAverage(PULSE_FILTER_TAPS);
    private final LinearFilter m_beltsCurrentFilter = LinearFilter.movingAverage(PULSE_FILTER_TAPS);

    private double m_filteredBeltsRPM;
    private double m_filteredBeltsStatorCurrent;
    private double m_beltsCommandVoltage;
    private boolean m_requestHopperPulse;
    private int m_pulseDebounceCycles;
    private int m_pulseCooldownCycles;

    // Creates a new ShooterFeeder
    public ShooterFeeder() {
        TalonFXConfiguration kickerConfigs = new TalonFXConfiguration();
        TalonFXConfiguration beltsConfigs = new TalonFXConfiguration();

        m_motorKicker = new TalonFX(Constants.SHOOTER_KICKER_CAN_ID);
        m_motorBelts = new TalonFX(Constants.SHOOTER_FEEDER_BELTS_CAN_ID);

        Slot0Configs kickerSlot0Configs = kickerConfigs.Slot0;
        kickerSlot0Configs.kP = K_P;
        kickerSlot0Configs.kI = K_I;
        kickerSlot0Configs.kD = K_D;
        kickerSlot0Configs.kV = K_FF * 60.0; // K_FF is in V/rpm, motor uses rps

        kickerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        beltsConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs kickerCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(KICKER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(KICKER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        kickerConfigs.withCurrentLimits(kickerCurrentLimits);

        CurrentLimitsConfigs beltsCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(BELTS_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(BELTS_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        beltsConfigs.withCurrentLimits(beltsCurrentLimits);

        m_motorKicker.getConfigurator().apply(kickerConfigs);
        m_motorKicker.setNeutralMode(NeutralModeValue.Coast);

        m_motorBelts.getConfigurator().apply(beltsConfigs);
        m_motorBelts.setNeutralMode(NeutralModeValue.Coast);

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
        m_filteredBeltsRPM = m_beltsRpmFilter.calculate(Math.abs(getBeltsRPM()));
        m_filteredBeltsStatorCurrent = m_beltsCurrentFilter.calculate(getBeltsStatorCurrent());
        updateHopperPulseRequest();

        SmartDashboard.putNumber("kicker/currentRPM", getKickerRPM());
        SmartDashboard.putNumber("kicker/goalRPM", m_goalRPM);
        SmartDashboard.putNumber("kicker/statorCurrent", getKickerStatorCurrent());
        SmartDashboard.putNumber("kicker/supplyCurrent", getKickerSupplyCurrent());

        SmartDashboard.putNumber("feeder/statorCurrent", getBeltsStatorCurrent());
        SmartDashboard.putNumber("feeder/supplyCurrent", getBeltsSupplyCurrent());
        SmartDashboard.putNumber("shooterFeeder/belts/currentRPM", getBeltsRPM());
        SmartDashboard.putNumber("shooterFeeder/belts/filteredRPM", m_filteredBeltsRPM);
        SmartDashboard.putNumber("shooterFeeder/belts/filteredStatorCurrent", m_filteredBeltsStatorCurrent);
        SmartDashboard.putNumber("feeder/voltage", m_motorBelts.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("shooterFeeder/onTarget", onTarget());
        SmartDashboard.putBoolean("shooterFeeder/requestHopperPulse", m_requestHopperPulse);
    }

    public double getKickerRPM() {
        return m_motorKicker.getVelocity().getValueAsDouble() * 60;
    }

    public double getKickerStatorCurrent() {
        return m_motorKicker.getStatorCurrent().getValueAsDouble();
    }

    public double getKickerSupplyCurrent() {
        return m_motorKicker.getSupplyCurrent().getValueAsDouble();
    }

    public double getBeltsStatorCurrent() {
        return m_motorBelts.getStatorCurrent().getValueAsDouble();
    }

    public double getBeltsSupplyCurrent() {
        return m_motorBelts.getSupplyCurrent().getValueAsDouble();
    }

    public double getBeltsRPM() {
        return m_motorBelts.getVelocity().getValueAsDouble() * 60;
    }

    public void setKickerVoltage(double voltage) {
        m_motorKicker.setControl(new VoltageOut(voltage).withEnableFOC(true));
    }

    public void setFeederBeltsVoltage(double voltage) {
        m_beltsCommandVoltage = voltage;
        m_beltsVoltageControl.Output = voltage;
        m_motorBelts.setControl(m_beltsVoltageControl);
    }

    public void setKickerRPM(double rpm) {
        m_goalRPM = rpm;
        m_velocityControl.Velocity = m_goalRPM / 60;
        m_velocityControl.FeedForward = K_FF * rpm;
        m_motorKicker.setControl(m_velocityControl);
    }

    public boolean onTarget() {
        return Math.abs(getKickerRPM() - m_goalRPM) < SPEED_TOLERANCE_RPM;
    }

    public void runFeederBelts() {
        setFeederBeltsVoltage(FEEDER_BELT_FEED_VOLTAGE);
    }

    public void runReverseUnjam() {
        setFeederBeltsVoltage(FEEDER_BELT_UNJAM_VOLTAGE);
    }

    public void stopFeederBelts() {
        setFeederBeltsVoltage(0.0);
    }

    public boolean shouldRequestHopperPulse() {
        return m_requestHopperPulse;
    }

    public void onHopperPulseTriggered() {
        m_requestHopperPulse = false;
        m_pulseDebounceCycles = 0;
        m_pulseCooldownCycles = PULSE_COOLDOWN_CYCLES;
    }

    public void stop() {
        m_motorKicker.setVoltage(0);
        m_motorBelts.setVoltage(0);
        m_goalRPM = 0.0;
        resetPulseDetection();
    }

    private void updateHopperPulseRequest() {
        if (m_pulseCooldownCycles > 0) {
            m_pulseCooldownCycles--;
            m_pulseDebounceCycles = 0;
            m_requestHopperPulse = false;
            return;
        }

        if (shouldStartPulseCandidate()) {
            m_pulseDebounceCycles++;
            if (m_pulseDebounceCycles >= PULSE_DEBOUNCE_CYCLES) {
                m_requestHopperPulse = true;
            }
        } else {
            m_pulseDebounceCycles = 0;
            m_requestHopperPulse = false;
        }
    }

    private boolean shouldStartPulseCandidate() {
        boolean beltsRunningForward = m_beltsCommandVoltage > 1.0;
        boolean beltsCurrentHigh = m_filteredBeltsStatorCurrent > BELTS_HIGH_STATOR_CURRENT_AMPS;
        boolean beltsRpmLow = m_filteredBeltsRPM < BELTS_LOW_RPM_THRESHOLD;

        return beltsRunningForward && (beltsCurrentHigh || beltsRpmLow);
    }

    private void resetPulseDetection() {
        m_beltsRpmFilter.reset();
        m_beltsCurrentFilter.reset();
        m_filteredBeltsRPM = 0.0;
        m_filteredBeltsStatorCurrent = 0.0;
        m_beltsCommandVoltage = 0.0;
        m_requestHopperPulse = false;
        m_pulseDebounceCycles = 0;
        m_pulseCooldownCycles = 0;
    }
}
