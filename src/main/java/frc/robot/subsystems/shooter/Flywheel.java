// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.BaseStatusSignal;
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
    private static final double SPEED_TOLERANCE_RPM = 175.0;
    private static final double DEFAULT_JAM_MIN_RPM = 250.0;
    private static final double DEFAULT_JAM_TIMEOUT_SEC = 1.0;
    private static final double DEFAULT_SHOT_DETECTED_MIN_RPM = 250.0;
    private static final double DEFAULT_SHOT_DETECTED_TORQUE_CURRENT_AMPS = 30.0;
    private static final double DEFAULT_SHOT_DETECTED_ARM_DELAY_SEC = 0.0;

    private static final String JAM_MIN_RPM_KEY = "flywheel/jamMinRPM";
    private static final String JAM_TIMEOUT_SEC_KEY = "flywheel/jamTimeoutSec";
    private static final String SHOT_DETECTED_MIN_RPM_KEY = "flywheel/shotDetectedMinRPM";
    private static final String SHOT_DETECTED_TORQUE_CURRENT_KEY = "flywheel/shotDetectedTorqueCurrentAmps";
    private static final String SHOT_DETECTED_ARM_DELAY_KEY = "flywheel/shotDetectedArmDelaySec";
    
    private final TalonFX m_motor;
    private final TalonFX m_follower;
    
    private static final double K_P = 0.4;       // tuned 3/25
    private static final double K_D = 0.0;       // D>0 does not help 3/25
    private static final double K_FF = 0.0019;  // V / rpm

    private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
    private static final Current STATOR_CURRENT_LIMIT = Amps.of(70);
        
    // VelocityControl instance which we reuse - saves some memory thrashing
    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut m_voltageControl = new VoltageOut(0);

    private double m_goalRPM;
    private double m_onTargetStartTimeSec;
    private boolean m_shotDetectionArmed = false;
    private boolean m_jamWatchActive = false;
    private boolean m_jamTimerActive = false;
    private double m_lastFuelPassTimeSec = 0.0;
    private boolean m_lastShotDetected = false;
    
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

        SmartDashboard.setDefaultNumber(JAM_MIN_RPM_KEY, DEFAULT_JAM_MIN_RPM);
        SmartDashboard.setDefaultNumber(JAM_TIMEOUT_SEC_KEY, DEFAULT_JAM_TIMEOUT_SEC);
        SmartDashboard.setDefaultNumber(SHOT_DETECTED_MIN_RPM_KEY, DEFAULT_SHOT_DETECTED_MIN_RPM);
        SmartDashboard.setDefaultNumber(SHOT_DETECTED_TORQUE_CURRENT_KEY, DEFAULT_SHOT_DETECTED_TORQUE_CURRENT_AMPS);
        SmartDashboard.setDefaultNumber(SHOT_DETECTED_ARM_DELAY_KEY, DEFAULT_SHOT_DETECTED_ARM_DELAY_SEC);

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
        updateShotDetectionArming();
        updateJamTimer();
        boolean shotDetected = isShotDetected();

        SmartDashboard.putNumber("flywheel/currentRPM", getRPM()); 
        SmartDashboard.putNumber("flywheel/goalRPM", m_goalRPM);
        SmartDashboard.putNumber("flywheel/rpmRatePerSec", getAccelerationRPMPerSec());
        SmartDashboard.putNumber("flywheel/leaderStator", m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/followerStator", m_follower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/leaderSupply", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/followerSupply", m_follower.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/leaderTorqueCurrent", m_motor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/followerTorqueCurrent", m_follower.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("flywheel/totalTorqueCurrent", getTotalTorqueCurrent());
        SmartDashboard.putBoolean("flywheel/jamDetectionArmed", isJamDetectionArmed());
        SmartDashboard.putBoolean("flywheel/shotDetectionArmed", m_shotDetectionArmed);
        SmartDashboard.putBoolean("flywheel/jamDetected", isJamDetected());
        SmartDashboard.putNumber("flywheel/timeSinceLastFuelSec", getTimeSinceLastFuelPassSec());
        SmartDashboard.putBoolean("flywheel/shotDetected", shotDetected);
    }
    
    public void setVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motor.setControl(m_voltageControl);
    }
    
    public double getRPM(){
        return m_motor.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
    }

    public double getGoalRPM() {
        return m_goalRPM;
    }

    public double getAccelerationRPMPerSec() {
        return m_motor.getAcceleration().getValueAsDouble() * 60.0;
    }

    public double getLatencyCompensatedRPM(double maxLatencySec) {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                m_motor.getVelocity(),
                m_motor.getAcceleration(),
                maxLatencySec) * 60.0;
    }

    public double getTotalTorqueCurrent() {
        return m_motor.getTorqueCurrent().getValueAsDouble() + m_follower.getTorqueCurrent().getValueAsDouble();
    }

    public void setRPM(double rpm) {
        setRPM(rpm, true);
    }

    public void setRPM(double rpm, boolean allowResetDetection) {
        if (allowResetDetection && Math.abs(rpm - m_goalRPM) > SPEED_TOLERANCE_RPM) {
            resetShotDetection();
            resetJamTimer();
        }

        m_goalRPM = rpm;

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

    public boolean isJamDetected() {
        return isJamDetectionArmed() && getTimeSinceLastFuelPassSec() >= getJamTimeoutSec();
    }

    public boolean isShotDetected() {
        return m_shotDetectionArmed && getTotalTorqueCurrent() > getShotDetectedTorqueCurrentThresholdAmps();
    }

    public void setJamWatchActive(boolean jamWatchActive) {
        if (m_jamWatchActive != jamWatchActive) {
            m_jamWatchActive = jamWatchActive;
            resetJamTimer();
        }
    }
        
    public void stop() { 
        setVoltage(0);
        m_goalRPM = 0;
        m_jamWatchActive = false;
        resetShotDetection();
        resetJamTimer();
    }

    private boolean isJamDetectionArmed() {
        return m_jamWatchActive
                && m_goalRPM >= getJamMinRPM();
    }

    private void updateJamTimer() {
        double now = Timer.getFPGATimestamp();
        boolean shotDetected = isShotDetected();

        if (!isJamDetectionArmed()) {
            resetJamTimer();
            m_lastShotDetected = shotDetected;
            return;
        }

        if (!m_jamTimerActive) {
            m_jamTimerActive = true;
            m_lastFuelPassTimeSec = now;
        }

        if (shotDetected && !m_lastShotDetected) {
            m_lastFuelPassTimeSec = now;
        }

        m_lastShotDetected = shotDetected;
    }

    private void updateShotDetectionArming() {
        double now = Timer.getFPGATimestamp();

        if (m_goalRPM < getShotDetectedMinRPM()) {
            resetShotDetection();
            m_onTargetStartTimeSec = now;
            return;
        }

        if (!m_shotDetectionArmed) {
            if (onTarget()) {
                if (now - m_onTargetStartTimeSec >= getShotDetectedArmDelaySec()) {
                    m_shotDetectionArmed = true;
                }
            } else {
                m_onTargetStartTimeSec = now;
            }
        }
    }

    private void resetShotDetection() {
        m_shotDetectionArmed = false;
        m_onTargetStartTimeSec = Timer.getFPGATimestamp();
    }

    private void resetJamTimer() {
        m_jamTimerActive = false;
        m_lastFuelPassTimeSec = Timer.getFPGATimestamp();
        m_lastShotDetected = false;
    }

    private double getJamMinRPM() {
        return SmartDashboard.getNumber(JAM_MIN_RPM_KEY, DEFAULT_JAM_MIN_RPM);
    }

    private double getJamTimeoutSec() {
        return Math.max(1.0 / Constants.ROBOT_FREQUENCY_HZ,
                SmartDashboard.getNumber(JAM_TIMEOUT_SEC_KEY, DEFAULT_JAM_TIMEOUT_SEC));
    }

    public double getTimeSinceLastFuelPassSec() {
        if (!m_jamTimerActive) {
            return 0.0;
        }

        return Timer.getFPGATimestamp() - m_lastFuelPassTimeSec;
    }

    private double getShotDetectedTorqueCurrentThresholdAmps() {
        return SmartDashboard.getNumber(SHOT_DETECTED_TORQUE_CURRENT_KEY, DEFAULT_SHOT_DETECTED_TORQUE_CURRENT_AMPS);
    }

    private double getShotDetectedMinRPM() {
        return SmartDashboard.getNumber(SHOT_DETECTED_MIN_RPM_KEY, DEFAULT_SHOT_DETECTED_MIN_RPM);
    }

    private double getShotDetectedArmDelaySec() {
        return Math.max(0.0, SmartDashboard.getNumber(SHOT_DETECTED_ARM_DELAY_KEY, DEFAULT_SHOT_DETECTED_ARM_DELAY_SEC));
    }
}
