// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//look for motor ratios
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Turret extends SubsystemBase {

  private Rotation2d m_goal = Rotation2d.fromDegrees(0.0);

  private final TalonFX m_turretMotor;

  private static final double SUPPLY_CURRENT_LIMIT = 60;
  private static final double STATOR_CURRENT_LIMIT = 40;

  private static final double K_P = 1.0; //TODO tune
  private static final double MAX_VEL_ROT_PER_SEC = 1;//TODO add new rotations/sec instead of meters
  private static final double MAX_ACC_ROT_PER_SEC_SQ = 0.5;
  private static final Rotation2d MAX_ROTATION = Rotation2d.fromRadians(Math.PI);
  private static final Rotation2d MIN_ROTATION = Rotation2d.fromRadians(-Math.PI);
  private static final int ENCODER_1_TOOTH_COUNT = 11;
  private static final int ENCODER_2_TOOTH_COUNT = 13;
  private static final int TURRET_TOOTH_COUNT = 100;
  
  private static final double GEAR_RATIO_MOTOR_TO_TURRET = 100/13;

  //three gear teeth, 100, 11, 13, chinese remainder theorme
  //motor gear to the 100 teeth ratio, use to scale readings
  //posiyion offset 

  /** Creates a new Turret. */
  public Turret() {
    m_turretMotor = new TalonFX(0); //TODO add the motor id constant
  
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    
    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;

    MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
    magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROT_PER_SEC; 
    magicConfigs.MotionMagicAcceleration = MAX_ACC_ROT_PER_SEC_SQ; 

    m_turretMotor.getConfigurator().apply(talonFXConfigs);
    m_turretMotor.setPosition(0); //make a seperae routuie
    //gearratio*rotations-position offset, offset is from chinese remainder theorem
    
    //go to position, but igure out how to go to position based on limitations
    //compute nearest position clockwise or counterclockwise with + or -, one side og 0 is -, other is +
    //whichever is closer if both follow rules 

    //later: motor encoded not to worry rn


    
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {    
    SmartDashboard.putNumber("turret/goalAngle", m_goal.getDegrees());
    SmartDashboard.putNumber("turret/currentAngle", getPosition().getDegrees());
    SmartDashboard.putNumber("turret/rawMotorAngle", m_turretMotor.getPosition().getValueAsDouble()*360);
    // SmartDashboard.putNumber("turret/chineseRemainderPosition", )//chinese )
  
    // This method will be called once per scheduler run
  }

  public void set(Rotation2d angle) {
    m_goal = limitRotation(angle);
    m_turretMotor.setControl(new MotionMagicVoltage(m_goal.getRotations()));
  }

  public Rotation2d getPosition(){
    return Rotation2d.fromRotations(m_turretMotor.getPosition().getValueAsDouble());
  }

    
//mathutil.clamp
//routine for motor yo turret
  private Rotation2d limitRotation(Rotation2d angle){
    return Rotation2d.fromRadians(MathUtil.clamp(angle.getDegrees(), MIN_ROTATION.getDegrees(), MAX_ROTATION.getRadians()));
  //this is for 30 degrees, want 360

  }
}