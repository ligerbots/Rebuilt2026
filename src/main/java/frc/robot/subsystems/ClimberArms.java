// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

    // TODO change to better number (currently filler number)
    private static final double MAX_VEL_ROTATION_PER_SEC = 5; //in RPS
    private static final double MAX_ACC_ROTATION_PER_SEC_SQUARE = 3; //in RPS^2

    private static final double ROTATIONS_PER_INCHES = 3; //TODO change to how many real rotation does it takes to extend 1 inch

    private static final double TOLERANCE = 0.1;//TODO change to a better tolerance number

    private double m_goalRotationLeft;
    private double m_goalDistanceLeft;
    private double m_currentRotationLeft;
    private double m_currentDistanceLeft;

    private double m_goalRotationRight;
    private double m_goalDistanceRight;
    private double m_currentRotationRight;
    private double m_currentDistanceRight;
    
    public static enum MotorSelection {
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
        magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROTATION_PER_SEC;
        magicConfigs.MotionMagicAcceleration = MAX_ACC_ROTATION_PER_SEC_SQUARE;

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
        getCurrentRotation(MotorSelection.LEFT);
        getCurrentRotation(MotorSelection.RIGHT);
        getCurrentDistance(MotorSelection.LEFT);
        getCurrentDistance(MotorSelection.RIGHT);

        SmartDashboard.putBoolean("ClimberArms/toTargetLeft", onTarget(MotorSelection.LEFT));
        SmartDashboard.putNumber("ClimberArms/currentRPMLeft", getRPM(m_leftMotor));
        SmartDashboard.putNumber("ClimberArms/goalDistanceLeft", m_goalDistanceLeft);
        SmartDashboard.putNumber("ClimberArms/goalRotationLeft", m_goalRotationLeft);
        SmartDashboard.putNumber("ClimberArms/currentRotationLeft", m_currentRotationLeft);
        
        SmartDashboard.putBoolean("ClimberArms/toTargetRight", onTarget(MotorSelection.RIGHT));
        SmartDashboard.putNumber("ClimberArms/currentRPMRight", getRPM(m_rightMotor));
        SmartDashboard.putNumber("ClimberArms/goalDistanceRight", m_goalDistanceRight);
        SmartDashboard.putNumber("ClimberArms/goalRotationRight", m_goalRotationRight);
        SmartDashboard.putNumber("ClimberArms/currentRotationRight", m_currentRotationRight);

    }

    private double getRPM(TalonFX motor) {
        return motor.getVelocity().getValueAsDouble() * 60; // convert rps to rpm
    }

    public void setPosition(double distance, MotorSelection selectedMotor, boolean loaded){
        int slotNumber;

        if (loaded) {
            slotNumber = 1;
        }else {
            slotNumber = 0;
        }

        if (selectedMotor == MotorSelection.LEFT) {
            m_goalDistanceLeft = distance;
            m_goalRotationLeft = distanceToRotation(distance);
            setRotation(m_goalRotationLeft, m_leftMotor, slotNumber);
        }else {
            m_goalDistanceRight = distance;
            m_goalRotationRight = distanceToRotation(distance);
            setRotation(m_goalRotationRight, m_rightMotor, slotNumber);
        }
        
    }

    private void setRotation(double rotation, TalonFX motor, int slotNumber) {
        motor.setControl(new MotionMagicVoltage(rotation).withSlot(slotNumber));
    }
    
    public double getCurrentRotation(MotorSelection selectedMotor){
        if (selectedMotor == MotorSelection.LEFT) {
            m_currentRotationLeft = m_leftMotor.getPosition().getValueAsDouble();
            return m_currentRotationLeft;
        }else {
            m_currentRotationRight = m_rightMotor.getPosition().getValueAsDouble();
            return m_currentRotationRight;
        }
    }

    /**
     * Gets the current rotation of the selected climber motor.
     *
     * @param selectedMotor which climber motor to read (LEFT or RIGHT)
     * @return the current motor position in rotations as reported by the TalonFX
     */

    public double getCurrentDistance(MotorSelection selectedMotor) {
        if (selectedMotor == MotorSelection.LEFT) {
            m_currentDistanceLeft = rotationToDistance(m_currentRotationLeft);
            return m_currentDistanceLeft;
        }else {
            m_currentDistanceRight = rotationToDistance(m_currentRotationRight);
            return m_currentDistanceRight;
        }

    }

    /**
     * Gets the current rotation of the selected climber motor.
     *
     * @param selectedMotor which climber motor to read (LEFT or RIGHT)
     * @return the current motor position in rotations as reported by the TalonFX
     */

    public boolean onTarget(MotorSelection selectedMotor){
        if (selectedMotor == MotorSelection.LEFT) {
            return Math.abs(m_currentRotationLeft - m_goalRotationLeft) < TOLERANCE;
        }else {
            return Math.abs(m_currentRotationRight - m_goalRotationRight) < TOLERANCE;
        }
        
    }

    /**
     * Checks whether the selected climber arm has reached its goal rotation.
     *
     * @param selectedMotor which climber motor to check (left or right)
     * @return {@code true} if the current rotation equals the goal rotation for the selected motor;
     *         {@code false} otherwise
     */
    
    private double distanceToRotation(double distance){
        return distance * ROTATIONS_PER_INCHES;
    }

    private double rotationToDistance(double rotation){
        return rotation / ROTATIONS_PER_INCHES;
    }
}
