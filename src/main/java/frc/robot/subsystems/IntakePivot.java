// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
    private static final double SUPPLY_CURRENT_LIMIT = 40;
    private static final double STATOR_CURRENT_LIMIT = 60;
    
    private static final double K_P = 15.0;
    private static final double K_P_HOLD = 3.0;
    
    private static final double MAX_VEL_ROT_PER_SEC = 20.0;
    private static final double MAX_ACC_ROT_PER_SEC2 = 50.0;
    
    private static final double ANGLE_TOLERANCE_DEG = 3.0;

    private static final double GEAR_RATIO = 1.0 / 24.0;
    
    private static final Rotation2d STOW_POSITION = Rotation2d.fromDegrees(-5.0);
    private static final Rotation2d DEPLOY_POSITION = Rotation2d.fromDegrees(75.0);

    private Rotation2d m_goal = Rotation2d.kZero;

    private final TalonFX m_pivotMotor;
    
    private static enum SlotNumber {
        MOVE(0),
        HOLD(1);

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
        slot0configs.kP = K_P;
        slot0configs.kI = 0.0;
        slot0configs.kD = 0.0;

        Slot1Configs slot1configs = talonFXConfigs.Slot1;
        slot1configs.kP = K_P_HOLD;
        slot1configs.kI = 0.0;
        slot1configs.kD = 0.0;
        
        MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        
        magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROT_PER_SEC;
        magicConfigs.MotionMagicAcceleration = MAX_ACC_ROT_PER_SEC2;
        
        m_pivotMotor.getConfigurator().apply(talonFXConfigs);
        m_pivotMotor.setPosition(0);
        m_pivotMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("intake/deployGoal", m_goal.getDegrees());
        SmartDashboard.putNumber("intake/deployAngle", getAngle().getDegrees());
        SmartDashboard.putNumber("intake/supplyCurrent", m_pivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("intake/statorCurrent", m_pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("intake/onTarget", onTarget());
        // SmartDashboard.putNumber("intake/rawMotorAngle",  m_pivotMotor.getPosition().getValueAsDouble());
    }
    
    public Command deployCommand() {
        return new InstantCommand(() -> setAngle(DEPLOY_POSITION), this)
                .andThen(new WaitUntilCommand(this::onTarget))
                .andThen(new InstantCommand(this::stop));
    }

    public Command stowCommand() {
        return new InstantCommand(() -> setAngle(STOW_POSITION), this)
                .andThen(new WaitUntilCommand(this::onTarget))
                .andThen(new InstantCommand(() -> setAngle(STOW_POSITION, SlotNumber.HOLD)));
    }

    public void setAngle(Rotation2d angle) {
        setAngle(angle, SlotNumber.MOVE);
    }

    private void setAngle(Rotation2d angle, SlotNumber slot) {
        m_goal = angle;
        m_pivotMotor.setControl(new MotionMagicVoltage(m_goal.getRotations() / GEAR_RATIO).withSlot(slot.getValue()));
    }
    
    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(m_pivotMotor.getPosition().getValueAsDouble() * GEAR_RATIO);
    }

    public void stop() {
        m_pivotMotor.setControl(new VoltageOut(0));
    }
    
    public boolean onTarget() {
        Rotation2d angle = getAngle();
        return Math.abs(angle.getDegrees() - m_goal.getDegrees()) < ANGLE_TOLERANCE_DEG;
    }

    public Command runPulse() {
        return deployCommand().andThen(new WaitUntilCommand(this::onTarget)).andThen(stowCommand()).andThen(new WaitUntilCommand(this::onTarget)).repeatedly();
    }
}
