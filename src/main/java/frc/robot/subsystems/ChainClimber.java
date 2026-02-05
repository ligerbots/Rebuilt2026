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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class ChainClimber extends SubsystemBase {

    private final TalonFX m_motor;
    private final TalonFX m_follower;

    // TODO need to calibrate the P value for the velocity loop, start small and increase until you get good response
    private static final double K_P = 3.0;
    private static final double K_P_LOADED = 3.0; 

    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = 60;

    // TODO change to better number (currently filler number)
    private static final double MAX_VEL_ROTATION_PER_SEC = 5; //in RPS
    private static final double MAX_ACC_ROTATION_PER_SEC_SQUARE = 3; //in RPS^2

    private static final double ROTATIONS_PER_INCHES = 3; //TODO change to how many real rotation does it takes to extend 1 inch

    private static final double TOLERANCE = 1;//TODO change to a better tolerance number


    private double m_goalDistance;

    private static enum SlotNumber {
        UNLOADED(0),
        LOADED(1);

        private final int value;

        SlotNumber(final int newValue) {
            value = newValue;
        }

        public int getValue() { return value; }

    }

    // private Follower

    // Creates a new ChainClimber
    public ChainClimber() {

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        m_motor = new TalonFX(Constants.CHAIN_CLIMBER_MOTOR_CAN_ID);
        m_follower = new TalonFX(Constants.CHAIN_CLIMBER_FOLLOWER_MOTOR_CAN_ID);

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
        slot1configs.kP = K_P_LOADED;  // start small!!!
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
        m_motor.getConfigurator().apply(talonFXConfigs);
        m_motor.setNeutralMode(NeutralModeValue.Brake);

        m_follower.getConfigurator().apply(talonFXConfigs);
        m_follower.setNeutralMode(NeutralModeValue.Brake);
        m_follower.setControl(new Follower(m_motor.getDeviceID(), null));

        m_motor.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ClimberArms/onTarget", onTarget());
        SmartDashboard.putNumber("ClimberArms/goalDistance", m_goalDistance);
        SmartDashboard.putNumber("ClimberArms/currentDistance", getCurrentDistance());
    }


    public void setDistance(double distance, boolean loaded){
        int slotNumber;

        if (loaded) {
            slotNumber = SlotNumber.LOADED.getValue();
        }else {
            slotNumber = SlotNumber.UNLOADED.getValue();;
        }

        m_goalDistance = distance;
        m_motor.setControl(new MotionMagicVoltage(distance).withSlot(slotNumber));
    }
    
    public double getCurrentDistance() {
      return m_motor.getPosition().getValueAsDouble();
    }

    public boolean onTarget(){
        return Math.abs(getCurrentDistance() - m_goalDistance) < TOLERANCE;
        
    }
}
