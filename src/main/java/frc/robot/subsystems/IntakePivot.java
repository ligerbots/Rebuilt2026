// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {

    /// deployed
    /// stowed
    /// poweredStow - low voltage
        ///-> Might need smaller P, seperate PID

    public enum IntakePivotState {
        DEPLOYED,
        STOWED,
        POWERED_STOW
    }

    public IntakePivotState m_currentState = IntakePivotState.STOWED;

    private Rotation2d m_goal = Rotation2d.kZero;

    private final TalonFX m_pivotMotor;
    
    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = 60;
    
    private static final double K_P_STOWED = 8.0;
    private static final double K_P_DEPLOYED = 15.0; // TODO TUNE
    
    private static final double MAX_VEL_ROT_PER_SEC = 20.0; // TODO change to more reasonable number (currently filler number)
    private static final double MAX_ACC_ROT_PER_SEC2 = 50.0; // TODO change to more reasonable number (currently filler number)
    
    private static final double GEAR_RATIO = 1.0 / 24.0;
    
    private static final Rotation2d STOW_POSITION = Rotation2d.kZero;
    private static final Rotation2d DEPLOY_POSITION = Rotation2d.fromDegrees(75.0);

    private static enum SlotNumber {
        STOWED(0),
        DEPLOYED(1);

        private final int value;

        SlotNumber(final int newValue) {
            value = newValue;
        }

        public int getValue() { return value; }
    }

    /** Creates a new IntakePivot. */
    public IntakePivot() {
        m_pivotMotor = new TalonFX(Constants.INTAKE_DEPLOY_ID);
        
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P_STOWED;
        slot0configs.kI = 0.0;
        slot0configs.kD = 0.0;

        Slot1Configs slot1configs = talonFXConfigs.Slot1;
        slot1configs.kV = 0.0; // A velocity target of 1 rps results in X V output
        slot1configs.kA = 0.0; // An acceleration of 1 rps/s requires X V output
        slot1configs.kP = K_P_DEPLOYED;  // start small!!!
        slot1configs.kI = 0.0; // no output for integrated error
        slot1configs.kD = 0.0; // A velocity error of 1 rps results in X V output
        
        MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        
        magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROT_PER_SEC;
        magicConfigs.MotionMagicAcceleration = MAX_ACC_ROT_PER_SEC2;
        
        m_pivotMotor.getConfigurator().apply(talonFXConfigs);
        m_pivotMotor.setPosition(0);
        m_pivotMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    
    @Override
    public void periodic() {

        switch (m_currentState) {

            case DEPLOYED:
                setAngle(DEPLOY_POSITION);
                break;

            case STOWED:
                setAngle(STOW_POSITION);
                break;

            case POWERED_STOW:
                setAngle(STOW_POSITION);
                break;
        }

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("intake/deployGoal", m_goal.getDegrees());
        SmartDashboard.putNumber("intake/deployAngle", getAngle().getDegrees());
        SmartDashboard.putNumber("intake/supplyCurrent", m_pivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("intake/statorCurrent", m_pivotMotor.getStatorCurrent().getValueAsDouble());

        // SmartDashboard.putNumber("intake/rawMotorAngle",  m_pivotMotor.getPosition().getValueAsDouble());
    }
    
    public void deploy() {
        setAngle(DEPLOY_POSITION);
    }

    public void stow() {
        setAngle(STOW_POSITION);
    }

    public void setAngle(Rotation2d angle) {
        m_goal = angle;
        int slot = (m_currentState == IntakePivotState.DEPLOYED) ? SlotNumber.DEPLOYED.getValue() : SlotNumber.STOWED.getValue();
        m_pivotMotor.setControl(new MotionMagicVoltage(m_goal.getRotations() / GEAR_RATIO).withSlot(slot));
    }
    
    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(m_pivotMotor.getPosition().getValueAsDouble() * GEAR_RATIO);
    }
}
