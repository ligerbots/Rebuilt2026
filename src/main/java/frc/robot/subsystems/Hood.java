// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Hood extends SubsystemBase {

  private Rotation2d m_goal = Rotation2d.fromDegrees(0.0);

  private final TalonFX m_hoodMotor;

  private static final double SUPPLY_CURRENT_LIMIT = 40;
  private static final double STATOR_CURRENT_LIMIT = 60;

  private static final double K_P = 1.0;

  private static final double MAX_VEL_RAD_PER_SEC = Units.degreesToRadians(50.0);
  private static final double MAX_ACC_RAD_PER_SEC = Units.degreesToRadians(50.0); // TODO change to better number (currently filler number)


  /** Creates a new Hood. */
  public Hood() {
    m_hoodMotor = new TalonFX(0); // FIXME change 0 to Constants.java value

    // set config to factory default
    
        
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    
    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;

    MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        
    magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_RAD_PER_SEC;
    magicConfigs.MotionMagicAcceleration = MAX_ACC_RAD_PER_SEC;

    m_hoodMotor.getConfigurator().apply(talonFXConfigs);
    m_hoodMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hood/goalAngle", m_goal.getDegrees());
    SmartDashboard.putNumber("hood/currentAngle", getPosition().getDegrees());
    SmartDashboard.putNumber("hood/rawMotorAngle",  m_hoodMotor.getPosition().getValueAsDouble());
   
  }

  public void set(Rotation2d angle) {
    m_goal = angle;
    m_hoodMotor.setControl(new MotionMagicVoltage(m_goal.getRotations()));
  }

  public Rotation2d getPosition(){
    return Rotation2d.fromRotations(m_hoodMotor.getPosition().getValueAsDouble());
  }
}
