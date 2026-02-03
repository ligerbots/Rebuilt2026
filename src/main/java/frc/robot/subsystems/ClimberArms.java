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

    private static final double TOLERANCE = 1;//TODO change to a better tolerance number


    private double m_goalDistanceLeft;
    private double m_goalDistanceRight;
    
    public static enum MotorSelection {
        LEFT,
        RIGHT
    }

    private static enum SlotNumber {
        ZERO(0),
        ONE(1);

        private final int value;

        SlotNumber(final int newValue) {
            value = newValue;
        }

        public int getValue() { return value; }

    }


    // Creates a new ClimberArms
    public ClimberArms() {

        //TODO comment out when no longer testing

        SmartDashboard.putNumber("ClimberArms/setLeft", 0);
        SmartDashboard.putNumber("ClimberArms/setRight", 0);

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

        talonFXConfigs.Feedback.withSensorToMechanismRatio(ROTATIONS_PER_INCHES); //pass down the conversion factor to motor

        // enable brake mode (after main config)
        m_leftMotor.getConfigurator().apply(talonFXConfigs);
        m_leftMotor.setNeutralMode(NeutralModeValue.Brake);

        m_rightMotor.getConfigurator().apply(talonFXConfigs);
        m_rightMotor.setNeutralMode(NeutralModeValue.Brake);

        m_leftMotor.setPosition(0);
        m_rightMotor.setPosition(0);
    }

    @Override
    public void periodic() {

        //TODO comment out when no longer testing

        setDistance(SmartDashboard.getNumber("ClimberArms/setLeft", 0), MotorSelection.LEFT, true);
        setDistance(SmartDashboard.getNumber("ClimberArms/setRight", 0), MotorSelection.RIGHT, true);

        // SmartDashboard.getNumber("ClimberArms/setRight", 0);

        SmartDashboard.putBoolean("ClimberArms/toTargetLeft", onTarget(MotorSelection.LEFT));
        SmartDashboard.putNumber("ClimberArms/goalDistanceLeft", m_goalDistanceLeft);
        SmartDashboard.putNumber("ClimberArms/currentDistanceLeft", getCurrentDistance(MotorSelection.LEFT));
        
        SmartDashboard.putBoolean("ClimberArms/toTargetRight", onTarget(MotorSelection.RIGHT));
        SmartDashboard.putNumber("ClimberArms/goalDistanceRight", m_goalDistanceRight);
        SmartDashboard.putNumber("ClimberArms/currentDistanceRight", getCurrentDistance(MotorSelection.RIGHT));

    }


    public void setDistance(double distance, MotorSelection selectedMotor, boolean loaded){
        int slotNumber;

        if (loaded) {
            slotNumber = SlotNumber.ONE.getValue();
        }else {
            slotNumber = SlotNumber.ZERO.getValue();
        }

        if (selectedMotor == MotorSelection.LEFT) {
            m_goalDistanceLeft = distance;
            m_leftMotor.setControl(new MotionMagicVoltage(distance).withSlot(slotNumber));
        }else {
            m_goalDistanceRight = distance;
            m_rightMotor.setControl(new MotionMagicVoltage(distance).withSlot(slotNumber));
        }
        
    }
    

    public double getCurrentDistance(MotorSelection selectedMotor) {
        if (selectedMotor == MotorSelection.LEFT) {
            return m_leftMotor.getPosition().getValueAsDouble();
        }else {
            return m_rightMotor.getPosition().getValueAsDouble();
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
            return Math.abs(getCurrentDistance(MotorSelection.LEFT) - m_goalDistanceLeft) < TOLERANCE;
        }else {
            return Math.abs(getCurrentDistance(MotorSelection.RIGHT) - m_goalDistanceRight) < TOLERANCE;
        }
        
    }

    /**
     * Checks whether the selected climber arm has reached its goal rotation.
     *
     * @param selectedMotor which climber motor to check (left or right)
     * @return {@code true} if the current rotation equals the goal rotation for the selected motor;
     *         {@code false} otherwise
     */
    
}
