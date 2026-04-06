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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    private enum RecoveryState {
        IDLE,
        STALL_DETECTING,
        REVERSING,
        DISARMED
    }
    private static final double SPEED_TOLERANCE_RPM = 100.0;    
    private static final double MIN_ACTIVE_GOAL_RPM = 500.0;
    private static final double AT_SPEED_RATIO = 0.8;
    private static final double STALL_RPM_RATIO = 0.55;
    private static final double STALL_CURRENT_AMPS = 35.0;
    private static final double STALL_DETECTION_SEC = 0.10;
    private static final double RECOVERY_REVERSE_VOLTAGE = -3.0;
    private static final double RECOVERY_REVERSE_SEC = 0.12;
    private static final double RECOVERY_DISARM_SEC = 0.18;
    
    private final TalonFX m_motor;
    private final TalonFX m_follower;
    
    private static final double K_P = 0.6;       // tuned 3/25
    private static final double K_D = 0.0;       // D>0 does not help 3/25
    private static final double K_FF = 0.00188;  // V / rpm

    private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
    private static final Current STATOR_CURRENT_LIMIT = Amps.of(60);
        
    // VelocityControl instance which we reuse - saves some memory thrashing
    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut m_voltageControl = new VoltageOut(0);

    private double m_goalRPM;
    private boolean m_seenNearTargetSinceSpinup = false;
    private RecoveryState m_recoveryState = RecoveryState.IDLE;
    private double m_stallStartSec = 0.0;
    private double m_recoveryPhaseEndSec = 0.0;
    
    // Creates a new FlyWheel
    public Flywheel() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  
        
        m_motor = new TalonFX(Constants.FLYWHEEL_CAN_ID);
        m_follower = new TalonFX(Constants.FLYWHEEL_FOLLOWER_CAN_ID);
        
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;
        slot0configs.kI = 0.0;
        slot0configs.kD = K_D;
        slot0configs.kV = K_FF * 60.0;   // K_FF is in V/rpm, motor uses rps

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        talonFXConfigs.withCurrentLimits(currentLimits);
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // enable coast mode (after main config)
        m_motor.getConfigurator().apply(talonFXConfigs);
        m_motor.setNeutralMode(NeutralModeValue.Coast);

        m_follower.getConfigurator().apply(talonFXConfigs);
        m_follower.setNeutralMode(NeutralModeValue.Coast);
        m_follower.setControl(new Follower(m_motor.getDeviceID(), MotorAlignmentValue.Opposed));

        // DO NOT mess with the update frequency on the motors. This can affect
        //  the leader/follower behavior. If we really need it, we will investigate.
        // https://www.chiefdelphi.com/t/setting-up-ctre-followers/512315/12
    }

    // private void optimizeCAN() {
    //     // For the motor, we want the RPM every loop
    //     m_motor.getVelocity().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
    //     m_motor.getStatorCurrent().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
    //     m_motor.optimizeBusUtilization();
    // }

    @Override
    public void periodic() {
        updateRecoveryState();

        if (m_goalRPM >= MIN_ACTIVE_GOAL_RPM && Math.abs(getRPM()) >= m_goalRPM * AT_SPEED_RATIO) {
            m_seenNearTargetSinceSpinup = true;
        }

        SmartDashboard.putNumber("flywheel/currentRPM", getRPM()); 
        SmartDashboard.putNumber("flywheel/goalRPM", m_goalRPM);
        SmartDashboard.putNumber("flywheel/leaderStator", m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/followerStator", m_follower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/leaderSupply", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/followerSupply", m_follower.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("flywheel/stallRecoveryActive", isRecovering());
    }
    
    public void setVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motor.setControl(m_voltageControl);
    }
    
    public double getRPM(){
        return m_motor.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
    }
    
    public void setRPM(double rpm) {
        if ((m_goalRPM < MIN_ACTIVE_GOAL_RPM && rpm >= MIN_ACTIVE_GOAL_RPM)
                || Math.abs(rpm - m_goalRPM) > 250.0
                || Math.signum(rpm) != Math.signum(m_goalRPM)) {
            m_seenNearTargetSinceSpinup = false;
            if (m_recoveryState == RecoveryState.STALL_DETECTING) {
                m_recoveryState = RecoveryState.IDLE;
                m_stallStartSec = 0.0;
            }
        }

        m_goalRPM = rpm;

        if (isRecovering()) {
            return;
        }

        m_velocityControl.Velocity = m_goalRPM / 60;
        m_motor.setControl(m_velocityControl);
    }
    
    /**
    * Checks if the shooter is at the target RPM within tolerance.
    * 
    * @param targetRpm The target shooter RPM
    * @return true if shooter is within tolerance, false otherwise
    */
    public boolean onTarget() {
        return Math.abs(getRPM() - m_goalRPM) < SPEED_TOLERANCE_RPM;
    }

    public boolean isRecovering() {
        return m_recoveryState != RecoveryState.IDLE;
    }
        
    public void stop() { 
        setVoltage(0);
        m_goalRPM = 0;
        m_seenNearTargetSinceSpinup = false;
        m_recoveryState = RecoveryState.IDLE;
        m_stallStartSec = 0.0;
        m_recoveryPhaseEndSec = 0.0;
    }

    private void updateRecoveryState() {
        double now = Timer.getFPGATimestamp();

        switch (m_recoveryState) {
            case IDLE:
            case STALL_DETECTING:
                detectStall(now);
                return;
            case REVERSING:
                if (now < m_recoveryPhaseEndSec) {
                    setVoltage(RECOVERY_REVERSE_VOLTAGE);
                    return;
                }

                m_recoveryState = RecoveryState.DISARMED;
                m_recoveryPhaseEndSec = now + RECOVERY_DISARM_SEC;
                setVoltage(0.0);
                return;
            case DISARMED:
                if (now < m_recoveryPhaseEndSec) {
                    setVoltage(0.0);
                    return;
                }

                m_recoveryState = RecoveryState.IDLE;
                m_recoveryPhaseEndSec = 0.0;
                return;
            default:
                return;
        }
    }

    private void detectStall(double now) {
        boolean stalled = m_goalRPM >= MIN_ACTIVE_GOAL_RPM
                && m_seenNearTargetSinceSpinup
                && Math.abs(getRPM()) <= m_goalRPM * STALL_RPM_RATIO
                && m_motor.getStatorCurrent().getValueAsDouble() >= STALL_CURRENT_AMPS;

        if (!stalled) {
            m_recoveryState = RecoveryState.IDLE;
            m_stallStartSec = 0.0;
            return;
        }

        if (m_recoveryState != RecoveryState.STALL_DETECTING) {
            m_recoveryState = RecoveryState.STALL_DETECTING;
            m_stallStartSec = now;
            return;
        }

        if (now - m_stallStartSec >= STALL_DETECTION_SEC) {
            m_recoveryState = RecoveryState.REVERSING;
            m_recoveryPhaseEndSec = now + RECOVERY_REVERSE_SEC;
            m_stallStartSec = 0.0;
            m_seenNearTargetSinceSpinup = false;
        }
    }
}
