// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//review
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;



public class Turret extends SubsystemBase {

  private Rotation2d m_goal = Rotation2d.fromDegrees(0.0);

  private final TalonFX m_hoodMotor;

  private static final double SUPPLY_CURRENT_LIMIT = 40;
  private static final double STATOR_CURRENT_LIMIT = 60;

  private static final double K_P = 1.0;
  private static final double MAX_VEL_METER_PER_SEC = Units.inchesToMeters(100.0);
  private static final double MAX_ACC_METER_PER_SEC_SQ = Units.inchesToMeters(750.0);
  private static final double MAX_JERK_METER_PER_SEC3 = Units.inchesToMeters(12500.0);
  private Rotation2d m_maxRotation2d = Rotation2d.fromRadians(360.0);
  private Rotation2d m_minRotation2d = Rotation2d.fromRadians(0.0);


  /** Creates a new Turret. */
  public Turret() {
    m_hoodMotor = new TalonFX(0); // FIXME change 0 to Constants.java value

    // set config to factory default
    m_hoodMotor.getConfigurator().apply(new TalonFXConfiguration());
        
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    
    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;

    m_hoodMotor.getConfigurator().apply(talonFXConfigs);
    m_hoodMotor.setPosition(0);

    MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        
    magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_METER_PER_SEC; // Target cruise velocity of 80 rps //TODO Change values TOTAL GUESSES 
    magicConfigs.MotionMagicAcceleration = MAX_ACC_METER_PER_SEC_SQ; // Target acceleration of 160 rps/s (0.5 seconds)
    magicConfigs.MotionMagicJerk = MAX_JERK_METER_PER_SEC3; // Target jerk of 1600 rps/s/s (0.1 seconds)
  
  }

  @Override
  public void periodic() {                                                 
    // This method will be called once per scheduler run
  }

  public void set(Rotation2d angle) {
    m_goal = angle;
    m_hoodMotor.setControl(new MotionMagicVoltage(m_goal.getRotations()));
  }

  public Rotation2d getPosition(){
    return Rotation2d.fromRotations(m_hoodMotor.getPosition().getValueAsDouble());
  }

   // set Motion Magic settings
    
//max rotation and min rotation in constants
//mathutil.clamp
public Rotation2d limiterRotation(Rotation2d angle){
  return Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), m_minRotation2d.getRadians(), m_maxRotation2d.getRadians()));
}
}