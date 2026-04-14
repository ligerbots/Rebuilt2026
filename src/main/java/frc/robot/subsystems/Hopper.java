// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    
    private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(35);
    private static final Current STATOR_CURRENT_LIMIT =  Amps.of(50);

    private static final double K_P = 0.1;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;
    private static final double K_FF = 0.0022;  // V/rpm

    private static final double INTAKE_RPM = 0.0;
    private static final double FEED_RPM = 3600.0;
    private static final double REVERSE_RPM = -3600.0;

    private static final String FEED_COMP_RPM_PER_MPS_KEY = "hopper/feedCompRPMPerMps";
    private static final String FEED_COMP_MAX_RPM_KEY = "hopper/feedCompMaxRPM";

    private static final double DEFAULT_FEED_COMP_RPM_PER_MPS = 450.0;
    private static final double DEFAULT_FEED_COMP_MAX_RPM = 1400.0;
    
    private final TalonFX m_motor;
    private final Supplier<ChassisSpeeds> m_robotRelativeSpeedsSupplier;

    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private double m_goalRPM;

    // Creates a new Hopper
    public Hopper() {
        this(null);
    }

    public Hopper(Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier) {
        m_robotRelativeSpeedsSupplier = robotRelativeSpeedsSupplier;
        m_motor = new TalonFX(Constants.HOPPER_TRANSFER_CAN_ID);

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

        // enable brake mode (after main config)
        m_motor.getConfigurator().apply(talonFXConfigs);
        m_motor.setNeutralMode(NeutralModeValue.Brake);

        if (Constants.OPTIMIZE_CAN) {
            optimizeCAN();
        }

        SmartDashboard.setDefaultNumber(FEED_COMP_RPM_PER_MPS_KEY, DEFAULT_FEED_COMP_RPM_PER_MPS);
        SmartDashboard.setDefaultNumber(FEED_COMP_MAX_RPM_KEY, DEFAULT_FEED_COMP_MAX_RPM);
    }

    private void optimizeCAN() {
        m_motor.getVelocity().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motor.getMotorVoltage().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motor.optimizeBusUtilization();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("hopper/RPM", getRPM());
        SmartDashboard.putNumber("hopper/goalRPM", m_goalRPM);
        SmartDashboard.putNumber("hopper/voltage", m_motor.getMotorVoltage().getValueAsDouble()); 
        SmartDashboard.putNumber("hopper/statorCurrent", m_motor.getStatorCurrent().getValueAsDouble()); 
        SmartDashboard.putNumber("hopper/supplyCurrent", m_motor.getSupplyCurrent().getValueAsDouble()); 
        SmartDashboard.putNumber("hopper/feedCompPerpendicularSpeedMps", getPerpendicularIntakeSpeedMetersPerSecond());
        SmartDashboard.putNumber("hopper/feedCompRPM", getFeedRPMCompensation());
    }
    
    public void intake(){
        setRPM(INTAKE_RPM);
    }
    
    public void feed(){
        setFeedRPM(FEED_RPM);
    }
    
    public void reverse() {
        setRPM(REVERSE_RPM);
    }

    public void stop(){
        setRPM(0.0);
    }
    
    public double getRPM(){
        return m_motor.getVelocity().getValueAsDouble() * 60; // convert rps to rpm
    }
    
    public void setRPM(double rpm) {
        m_goalRPM = rpm;
        m_velocityControl.Velocity = m_goalRPM / 60.0;
        m_motor.setControl(m_velocityControl);
    }

    public void setFeedRPM(double baseRPM) {
        if (baseRPM <= 0.0) {
            setRPM(baseRPM);
            return;
        }

        setRPM(baseRPM + getFeedRPMCompensation());
    }

    private double getFeedRPMCompensation() {
        double perpendicularSpeedMetersPerSecond = getPerpendicularIntakeSpeedMetersPerSecond();
        double compensationRPM = -perpendicularSpeedMetersPerSecond
                * SmartDashboard.getNumber(FEED_COMP_RPM_PER_MPS_KEY, DEFAULT_FEED_COMP_RPM_PER_MPS);

        double maxCompensationRPM = Math.abs(
                SmartDashboard.getNumber(FEED_COMP_MAX_RPM_KEY, DEFAULT_FEED_COMP_MAX_RPM));
        return MathUtil.clamp(compensationRPM, -maxCompensationRPM, maxCompensationRPM);
    }

    private double getPerpendicularIntakeSpeedMetersPerSecond() {
        if (m_robotRelativeSpeedsSupplier == null) {
            return 0.0;
        }

        ChassisSpeeds robotRelativeSpeeds = m_robotRelativeSpeedsSupplier.get();
        if (robotRelativeSpeeds == null) {
            return 0.0;
        }

        // Intake is on the front of the robot, so the perpendicular component is
        // just robot-relative forward/backward velocity.
        return robotRelativeSpeeds.vxMetersPerSecond;
    }
}
