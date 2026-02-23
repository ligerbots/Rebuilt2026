// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Hood extends SubsystemBase {

    private static final double ANGLE_TOLERANCE_DEG = 2.0;

    private static final double MIN_ANGLE_DEG = 0.0;
    private static final double MAX_ANGLE_DEG = 25.0;

    private static final double GEAR_RATIO = 12.0/36.0 * 15.0/24.0 * 10.0/174.0;
    
    private static final double SUPPLY_CURRENT_LIMIT = 20;
    private static final double STATOR_CURRENT_LIMIT = 30;
    
    private static final double K_P = 5.0;
    
    // private static final double MAX_VEL_ROT_PER_SEC = 12.0 / GEAR_RATIO;
    // private static final double MAX_ACC_ROT_PER_SEC = 20.0 / GEAR_RATIO;
    //
    // 2/14 testing at WPI - slowed down; jumping off end. Still needed??
    private static final double MAX_VEL_ROT_PER_SEC = 8.0 / GEAR_RATIO;
    private static final double MAX_ACC_ROT_PER_SEC = 15.0 / GEAR_RATIO;

    private final TalonFX m_motor;
    private double m_goalDeg = 0.0;

    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0);    
    
    /** Creates a new Hood. */
    public Hood() {
        m_motor = new TalonFX(Constants.HOOD_CAN_ID);
        
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;
        slot0configs.kI = 0.0;
        slot0configs.kD = 0.0;
        
        // talonFXConfigs.Feedback.withSensorToMechanismRatio(1/GEAR_RATIO); //pass down the conversion factor to motor

        MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        
        magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROT_PER_SEC;
        magicConfigs.MotionMagicAcceleration = MAX_ACC_ROT_PER_SEC;
                
        m_motor.getConfigurator().apply(talonFXConfigs);
        m_motor.setPosition(0);

        if (Constants.OPTIMIZE_CAN) {
            optimizeCAN();
        }
    }

    private void optimizeCAN() {
        m_motor.getPosition().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_motor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("hood/goalAngle", m_goalDeg);
        SmartDashboard.putNumber("hood/currentAngle", getAngle().getDegrees());
        // SmartDashboard.putNumber("hood/rawMotorAngle",  m_motor.getPosition().getValueAsDouble());
    }
    
    public void setAngle(Rotation2d angle) {
        m_goalDeg = MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_DEG, MAX_ANGLE_DEG);

        m_positionControl.Position = m_goalDeg / 360.0 / GEAR_RATIO;
        m_motor.setControl(m_positionControl);
    }
    
    public Rotation2d getAngle() {
        double rot = m_motor.getPosition().getValueAsDouble();
        return Rotation2d.fromRotations(rot * GEAR_RATIO);
    }
    
    /**
    * Checks if the hood is at the target angle within tolerance.
    * 
    * @param targetAngle The target hood angle as a Rotation2d object
    * @return true if hood is within tolerance, false otherwise
    */
    public boolean onTarget() {
        double angleDeg = getAngle().getDegrees();
        return Math.abs(angleDeg - m_goalDeg) < ANGLE_TOLERANCE_DEG;
    }
}
