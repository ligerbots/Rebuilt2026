// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class ClimberArms extends SubsystemBase {

    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;

    // TODO need to calibrate the P value for the velocity loop, start small and increase until you get good response
    private static final double K_P = 3.0;
    private static final double K_P_DOWN = 3.0; 

    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = 60;

    private static final double MAX_VEL_RAD_PER_SEC = Units.degreesToRadians(50.0);
    private static final double MAX_ACC_RAD_PER_SEC = Units.degreesToRadians(50.0); // TODO change to better number (currently filler number)

    private static final double ROTATIONS_TO_INCHES = 3; //TODO change to how many real rotation does it takes to extend 1 inch

    private double m_goalRotation_L;
    private double m_goalDistance_L;
    private double m_currentRotation_L;
    private double m_currentDistance_L;

    private double m_goalRotation_R;
    private double m_goalDistance_R;
    private double m_currentRotation_R;
    private double m_currentDistance_R;
    
    public static enum motorSelection {
        LEFT,
        RIGHT
    }


    // Creates a new ClimberArms
    public ClimberArms() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        m_leftMotor = new TalonFX(Constants.CLIMBER_LEFT_MOTOR_CAN_ID);
        m_rightMotor = new TalonFX(Constants.CLIMBER_RIGHT_MOTOR_CAN_ID);

        //TODO find out good K values for each term
        //set slot0 for unloaded state
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kV = 0.0; // A velocity target of 1 rps results in X V output
        slot0configs.kA = 0.0; // An acceleration of 1 rps/s requires X V output
        slot0configs.kP = K_P;  // start small!!!
        slot0configs.kI = 0.0; // no output for integrated error
        slot0configs.kD = 0.0; // A velocity error of 1 rps results in X V output

        //set slot1 for loaded state
        Slot1Configs slot1configs = talonFXConfigs.Slot1;
        slot1configs.kV = 0.0; // A velocity target of 1 rps results in X V output
        slot1configs.kA = 0.0; // An acceleration of 1 rps/s requires X V output
        slot1configs.kP = K_P_DOWN;  // start small!!!
        slot1configs.kI = 0.0; // no output for integrated error
        slot1configs.kD = 0.0; // A velocity error of 1 rps results in X V output

        //MotionMagic configs
        MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_RAD_PER_SEC;
        magicConfigs.MotionMagicAcceleration = MAX_ACC_RAD_PER_SEC;

        //Apply current limits
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
        //get current position
        getCurrentRotation(motorSelection.LEFT);
        getCurrentRotation(motorSelection.RIGHT);
        getCurrentDistance(motorSelection.LEFT);
        getCurrentDistance(motorSelection.RIGHT);

        SmartDashboard.putBoolean("ClimberArms/toTarget_L", onTarget(motorSelection.LEFT));
        SmartDashboard.putNumber("ClimberArms/currentRPM_L", getRPM(m_leftMotor));
        SmartDashboard.putNumber("ClimberArms/goalDistance_L", m_goalDistance_L);
        SmartDashboard.putNumber("ClimberArms/goalRotation_L", m_goalRotation_L);
        SmartDashboard.putNumber("ClimberArms/currentRotation_L", m_currentRotation_L);
        
        SmartDashboard.putBoolean("ClimberArms/toTarget_R", onTarget(motorSelection.RIGHT));
        SmartDashboard.putNumber("ClimberArms/currentRPM_R", getRPM(m_rightMotor));
        SmartDashboard.putNumber("ClimberArms/goalDistance_R", m_goalDistance_R);
        SmartDashboard.putNumber("ClimberArms/goalRotation_R", m_goalRotation_R);
        SmartDashboard.putNumber("ClimberArms/currentRotation_R", m_currentRotation_R);

    }

    public double getRPM(TalonFX motor) {
        return motor.getVelocity().getValueAsDouble() * 60; // convert rps to rpm
    }

    public void setPosition(double distance, motorSelection selectedMotor, boolean loaded){
        int slotNumber;

        if (loaded) {
            slotNumber = 1;
        }else {
            slotNumber = 0;
        }

        if (selectedMotor == motorSelection.LEFT) {
            m_goalDistance_L = distance;
            m_goalRotation_L = distanceToRotation(distance);
            setRotation(m_goalRotation_L, m_leftMotor, slotNumber);
        }else {
            m_goalDistance_R = distance;
            m_goalRotation_R = distanceToRotation(distance);
            setRotation(m_goalRotation_R, m_rightMotor, slotNumber);
        }
        
    }

    private void setRotation(double rotation, TalonFX motor, int slotNumber) {
        motor.setControl(new MotionMagicVoltage(rotation).withSlot(slotNumber));
    }
    
    public double getCurrentRotation(motorSelection selectedMotor){
        if (selectedMotor == motorSelection.LEFT) {
            m_currentRotation_L = m_leftMotor.getPosition().getValueAsDouble();
            return m_currentRotation_L;
        }else {
            m_currentRotation_R = m_leftMotor.getPosition().getValueAsDouble();
            return m_currentRotation_R;
        }
    }

    public double getCurrentDistance(motorSelection selectedMotor) {
        if (selectedMotor == motorSelection.LEFT) {
            m_currentDistance_L = rotationToDistance(m_currentRotation_L);
            return m_currentDistance_L;
        }else {
            m_currentDistance_R = rotationToDistance(m_currentRotation_R);
            return m_currentDistance_R;
        }

    }

    public boolean onTarget(motorSelection selectedMotor){
        if (selectedMotor == motorSelection.LEFT) {
            return m_currentRotation_L == m_goalRotation_L;
        }else {
            return m_currentRotation_R == m_goalRotation_R;
        }
        
    }
    
    private double distanceToRotation(double distance){
        return distance * ROTATIONS_TO_INCHES;
    }

    private double rotationToDistance(double rotation){
        return rotation / ROTATIONS_TO_INCHES;
    }
}
