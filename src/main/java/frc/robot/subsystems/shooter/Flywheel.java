// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import java.util.ArrayDeque;
import java.util.Deque;

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
import frc.robot.utilities.RobotLog;

public class Flywheel extends SubsystemBase {
    private static final double SPEED_TOLERANCE_RPM = 175.0;
    private static final double JAM_MIN_GOAL_RPM = 250.0;
    private static final double JAM_SPINUP_GRACE_SEC = 0.05;
    private static final double JAM_SMOOTHING_WINDOW_SEC = 1.0;
    private static final double SHOT_DETECTED_TORQUE_CURRENT_AMPS = 30.0;
    private static final double SHOT_DETECTED_ARM_DELAY_SEC = 0.1;

    private record TorqueCurrentSample(double timestampSec, double currentAmps) {}
    
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
    private double m_jamGrace;
    private double m_lastSpinupCommandTimeSec;
    private double m_onTargetStartTimeSec;
    private boolean m_shotDetectionArmed = false;
    private final Deque<TorqueCurrentSample> m_torqueCurrentSamples = new ArrayDeque<>();
    private double m_torqueCurrentSampleSum = 0.0;
    private double JAM_GRACE = 0.75;
    
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
        double now = Timer.getFPGATimestamp();

        updateTorqueCurrentAverage(now);
        updateShotDetectionArming(now);
        updateJamTime(now);        

        // Driver-facing status
        SmartDashboard.putBoolean("flywheel/shotDetected", isShotDetected());
        SmartDashboard.putBoolean("flywheel/jammed", isCurrentJamDetected());

        // Speed and setpoint
        RobotLog.log("flywheel/currentRPM", getRPM());
        RobotLog.log("flywheel/goalRPM", m_goalRPM);

        // Detection state
        RobotLog.log("flywheel/shotDetectionArmed", m_shotDetectionArmed);
        RobotLog.log("flywheel/jamDetectionArmed", isJamDetectionArmed(now));
        RobotLog.log("flywheel/jamGrace", m_jamGrace);
        RobotLog.log("flywheel/currentJamDetected", isCurrentJamDetected());

        // Motor electrical data
        RobotLog.log("flywheel/leaderStator", m_motor.getStatorCurrent().getValueAsDouble());
        RobotLog.log("flywheel/followerStator", m_follower.getStatorCurrent().getValueAsDouble());
        RobotLog.log("flywheel/leaderSupply", m_motor.getSupplyCurrent().getValueAsDouble());
        RobotLog.log("flywheel/followerSupply", m_follower.getSupplyCurrent().getValueAsDouble());
        RobotLog.log("flywheel/leaderTorqueCurrent", m_motor.getTorqueCurrent().getValueAsDouble());
        RobotLog.log("flywheel/followerTorqueCurrent", m_follower.getTorqueCurrent().getValueAsDouble());

        // Derived current metrics
        RobotLog.log("flywheel/totalTorqueCurrent", getTotalTorqueCurrent());
        RobotLog.log("flywheel/avgTorqueCurrent", getAverageTorqueCurrent());
    }
    
    public void setVoltage(double voltage) {
        m_voltageControl.Output = voltage;
        m_motor.setControl(m_voltageControl);
    }
    
    public double getRPM(){
        return m_motor.getVelocity().getValueAsDouble() * 60; //convert rps to rpm
    }

    public double getTotalTorqueCurrent() {
        return m_motor.getTorqueCurrent().getValueAsDouble() + m_follower.getTorqueCurrent().getValueAsDouble();
    }

    public double getAverageTorqueCurrent() {
        if (m_torqueCurrentSamples.isEmpty()) {
            return 0.0;
        }

        return m_torqueCurrentSampleSum / m_torqueCurrentSamples.size();
    }
    
    public void setRPM(double rpm) {
        if (Math.abs(rpm - m_goalRPM) > SPEED_TOLERANCE_RPM) {
            double now = Timer.getFPGATimestamp();
            m_lastSpinupCommandTimeSec = now;
            resetTorqueCurrentAverage();
            resetShotDetection(now);
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

    public boolean isShotDetected() {
        return m_shotDetectionArmed && getTotalTorqueCurrent() > getShotDetectedTorqueCurrentThresholdAmps();
    }
        
    public void stop() { 
        setVoltage(0);
        m_goalRPM = 0;
        resetTorqueCurrentAverage();
        resetShotDetection(Timer.getFPGATimestamp());
    }

    private boolean isJamDetectionArmed(double now) {
        return m_goalRPM >= getJamMinGoalRPM()
                && now - m_lastSpinupCommandTimeSec >= getJamSpinupGraceSec()
                && hasFilledSmoothingWindow(now);
    }

    private void updateTorqueCurrentAverage(double now) {
        double current = getTotalTorqueCurrent();
        m_torqueCurrentSamples.addLast(new TorqueCurrentSample(now, current));
        m_torqueCurrentSampleSum += current;
        pruneTorqueCurrentSamples(now);
    }

    private void resetTorqueCurrentAverage() {
        m_torqueCurrentSamples.clear();
        m_torqueCurrentSampleSum = 0.0;
    }

    private void pruneTorqueCurrentSamples(double now) {
        double windowSec = getJamSmoothingWindowSec();

        while (!m_torqueCurrentSamples.isEmpty()
                && now - m_torqueCurrentSamples.peekFirst().timestampSec() > windowSec) {
            m_torqueCurrentSampleSum -= m_torqueCurrentSamples.removeFirst().currentAmps();
        }
    }

    private boolean hasFilledSmoothingWindow(double now) {
        return !m_torqueCurrentSamples.isEmpty()
                && now - m_torqueCurrentSamples.peekFirst().timestampSec() >= getJamSmoothingWindowSec();
    }

    private void updateJamTime(double now){
        if (isShotDetected() == true){
                m_jamGrace = now;
            }
        }

    public boolean isCurrentJamDetected() {
        double now = Timer.getFPGATimestamp();

        return (now - m_jamGrace) >= JAM_GRACE && m_shotDetectionArmed;
    }

    private void updateShotDetectionArming(double now) {
        if (m_goalRPM < getJamMinGoalRPM()) {
            resetShotDetection(now);
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

    private void resetShotDetection(double now) {
        m_shotDetectionArmed = false;
        m_onTargetStartTimeSec = now;
    }

    private double getJamMinGoalRPM() {
        return JAM_MIN_GOAL_RPM;
    }

    private double getJamSpinupGraceSec() {
        return JAM_SPINUP_GRACE_SEC;
    }

    private double getJamSmoothingWindowSec() {
        return Math.max(1.0 / Constants.ROBOT_FREQUENCY_HZ, JAM_SMOOTHING_WINDOW_SEC);
    }

    private double getShotDetectedTorqueCurrentThresholdAmps() {
        return SHOT_DETECTED_TORQUE_CURRENT_AMPS;
    }

    private double getShotDetectedArmDelaySec() {
        return SHOT_DETECTED_ARM_DELAY_SEC;
    }
}
