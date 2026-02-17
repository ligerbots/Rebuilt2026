// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    
    // STOW is public so Intake can handle the command
    public static final Rotation2d STOW_POSITION = Rotation2d.fromDegrees(-5.0);
    private static final Rotation2d DEPLOY_POSITION = Rotation2d.fromDegrees(75.0);

    private static final Rotation2d PULSE_POSITION = Rotation2d.fromDegrees(10.0);

    private Rotation2d m_goal = Rotation2d.kZero;

    private final TalonFX m_motor;
    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0);
    private final VoltageOut m_stopControl = new VoltageOut(0);

    public static enum SlotNumber {
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
        m_motor = new TalonFX(Constants.INTAKE_DEPLOY_ID);
        
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
        
        m_motor.getConfigurator().apply(talonFXConfigs);
        m_motor.setPosition(0);
        m_motor.setNeutralMode(NeutralModeValue.Coast);

        if (Constants.OPTIMIZE_CAN) {
            optimizeCAN();
        }
    }

    private void optimizeCAN() {
        m_motor.getPosition().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motor.getMotorVoltage().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("intake/deployGoal", m_goal.getDegrees());
        SmartDashboard.putNumber("intake/deployAngle", getAngle().getDegrees());
        SmartDashboard.putNumber("intake/supplyCurrent", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("intake/statorCurrent", m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("intake/onTarget", onTarget());
        // SmartDashboard.putNumber("intake/rawMotorAngle",  m_pivotMotor.getPosition().getValueAsDouble());
    }
    
    public void setAngle(Rotation2d angle) {
        setAngle(angle, SlotNumber.MOVE);
    }

    public void holdAngle(Rotation2d angle) {
        setAngle(angle, SlotNumber.HOLD);
    }

    private void setAngle(Rotation2d angle, SlotNumber slot) {
        m_goal = angle;

        m_positionControl.Position = m_goal.getRotations() / GEAR_RATIO;
        m_positionControl.Slot = slot.getValue();
        m_motor.setControl(m_positionControl);
    }
    
    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(m_motor.getPosition().getValueAsDouble() * GEAR_RATIO);
    }

    public void stop() {
        m_motor.setControl(m_stopControl);
    }
    
    public boolean onTarget() {
        Rotation2d angle = getAngle();
        return Math.abs(angle.getDegrees() - m_goal.getDegrees()) < ANGLE_TOLERANCE_DEG;
    }

    // Note: stowCommand is in Intake, since it also involves the Rollers
    
    public Command runPulseCommand() {
        // This can be killed, since WaitCommand always finishes.
        return new InstantCommand(() -> setAngle(PULSE_POSITION), this)
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(() -> setAngle(STOW_POSITION), this))
            .andThen(new WaitCommand(0.5))
            .repeatedly();
    }
    
    public Command deployCommand() {
        Command cmd = new InstantCommand(() -> setAngle(DEPLOY_POSITION))
                .andThen(new WaitUntilCommand(this::onTarget))
                .andThen(new InstantCommand(this::stop));
        // Add a requirement on the entire command (including WaitUntilCommand, we hope).
        // Then, if it gets stuck in WaitUntilCommand, another Pivot command will still kill it.
        cmd.addRequirements(this);
        return cmd;
    }
}
