// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWheels extends SubsystemBase {
    public enum IntakeWheelsState {
        STOW, DEPLOY
    }

    //IMPORTANT NOTE: All angles are relitive to stowed which is zero in the code.

    private static final double ANGLE_TOLERANCE_DEG = 8.0;

    private static final int PIVOT_CURRENT_LIMIT = 40;
    private static final int ROLLER_CURRENT_LIMIT = 60;

    private static final double GEAR_RATIO = 16.0/42.0 * 1.0/4.0; 

    // Constants for the pivot PID controller
    private static final double K_P = 4;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;

    private final SparkMax m_pivotMotor;
    private final SparkFlex m_rollerMotor;

    // private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pivotController;

    private final Rotation2d STOWED_ANGLE = Rotation2d.fromDegrees(0.0);
    private final Rotation2d DEPLOYED_ANGLE = Rotation2d.fromDegrees(115.0); 

    private final double ROLLER_INTAKE_SPEED = 0.6/2.0;
    private final double ROLLER_OUTTAKE_SPEED = -1.0/2.0; 
    private final double ROLLER_HOLD_SPEED = 0.5; 

    // This is RPM
    private static final double STALL_VELOCITY_LIMIT = 10; 

    private IntakeWheelsState m_state = IntakeWheelsState.STOW;

    private double m_goalAngle; //Used for readout in elastic only

    //Trapisoidl:
    private static final double MAX_VEL_ROT_PER_SEC = 3;
    private static final double MAX_ACC_ROT_PER_SEC2 = 2;
    private static final double ROBOT_LOOP_PERIOD = 0.02;

    // Trapezoid Profile
    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL_ROT_PER_SEC, MAX_ACC_ROT_PER_SEC2));
    private State m_currentState = new State();

    // Used for detecting if we have coral based on speed. Size should be: 1 / robot loop period(0.02 seconds) * 2 * Duration before we decide we have coral
   private final MedianFilter m_coralHoldFilter = new MedianFilter(10);

    // Construct a new shooterPivot subsystem
    public IntakeWheels() {
        m_pivotMotor = new SparkMax(Constants.CORAL_GROUND_PIVOT_ID, MotorType.kBrushless);
        m_rollerMotor = new SparkFlex(Constants.CORAL_GROUND_ROLLER_ID, MotorType.kBrushless);

        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig.inverted(false);
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
        pivotMotorConfig.encoder.positionConversionFactor(GEAR_RATIO);

        pivotMotorConfig.closedLoop.p(K_P).i(K_I).d(K_D);

        pivotMotorConfig.closedLoop.outputRange(-1, 1);
        pivotMotorConfig.closedLoop.positionWrappingEnabled(false); // don't treat it as a circle

        m_pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pivotMotor.getEncoder().setPosition(0);

        SparkMaxConfig rollerMotor = new SparkMaxConfig();
        rollerMotor.inverted(true);
        rollerMotor.idleMode(IdleMode.kBrake);
        rollerMotor.smartCurrentLimit(ROLLER_CURRENT_LIMIT);

        m_rollerMotor.configure(rollerMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // controller for PID control
        m_pivotController = m_pivotMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        boolean stalled = m_coralHoldFilter.calculate(Math.abs(getRollerSpeed().getRotations())) < STALL_VELOCITY_LIMIT;
        SmartDashboard.putNumber("IntakeWheels/goalAngle", m_goalAngle);
        SmartDashboard.putNumber("IntakeWheels/angleOffFromGoal", m_goalAngle - getPivotAngle().getDegrees());
        SmartDashboard.putBoolean("IntakeWheels/isStalled", stalled);
        SmartDashboard.putNumber("IntakeWheels/currentAngle", getPivotAngle().getDegrees());
        SmartDashboard.putString("IntakeWheels/state", m_state.toString());
        //Use triggers to intake/outtake
        switch (m_state) {
            case STOW:
                setAngleWithProfile(STOWED_ANGLE);
                setRollerSpeedPercent(0);
                break;
            case DEPLOY:
                setRollerSpeedPercent(ROLLER_INTAKE_SPEED);
                setAngleWithProfile(DEPLOYED_ANGLE);

                if ((0 <= (getPivotAngle().getDegrees() - DEPLOYED_ANGLE.getDegrees() + ANGLE_TOLERANCE_DEG)) && stalled) {
                    deploy();
                    // System.out.println("Stalled, intaking");
                }
                break;
        }
    }

    private void setAngleWithProfile(Rotation2d angle) {
        m_goalAngle = angle.getDegrees();

        State goalState = new State(angle.getRotations(), 0);

        // Trapezoid Profile
        m_currentState = m_profile.calculate(ROBOT_LOOP_PERIOD, m_currentState, goalState);

        m_pivotController.setReference(m_currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public Rotation2d getRollerSpeed() {
        return Rotation2d.fromRotations(m_rollerMotor.getEncoder().getVelocity());
    }

    // Get the current pivot angle
    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(m_pivotMotor.getEncoder().getPosition());
    }

    public void setPivotAngle(Rotation2d angle, double feedForward) {
        m_goalAngle = angle.getDegrees();
        m_pivotController.setReference(angle.getRotations(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
        // SmartDashboard.putNumber("pivot/goal", m_goal.getDegrees());
    }

    // set the speed of the roller in RPM
    public void setRollerSpeedPercent(double speed) {
        m_rollerMotor.set(speed);
    }

    // state changes
    public void stow() {
        m_state = IntakeWheelsState.STOW;
    }
    public void deploy() {
        m_state = IntakeWheelsState.DEPLOY;
    }
    public IntakeWheelsState getState() {
        return m_state;
    }
}