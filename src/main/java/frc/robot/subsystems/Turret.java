// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//look for motor ratios
package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utilities.ChineseRemainder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class Turret extends SubsystemBase {

  private Rotation2d m_goal = Rotation2d.fromDegrees(0.0);

  private final TalonFX m_turretMotor;
  private final CANcoder m_thrubore1; 
  private final CANcoder m_thrubore2; 


  private static final double SUPPLY_CURRENT_LIMIT = 60;
  private static final double STATOR_CURRENT_LIMIT = 40;

  private static final double K_P = 1.0; //TODO tune
  private static final double MAX_VEL_ROT_PER_SEC = 1;//TODO add new rotations/sec instead of meters
  private static final double MAX_ACC_ROT_PER_SEC_SQ = 0.5;
  private static final Rotation2d MAX_ROTATION = Rotation2d.fromRadians(Math.PI);
  private static final Rotation2d MIN_ROTATION = Rotation2d.fromRadians(-Math.PI);
  //private static final double ENCODER_1_TOOTH_COUNT = 11;
  //private static final double ENCODER_2_TOOTH_COUNT = 13;
  //private static final double TURRET_TOOTH_COUNT = 100;
  private static final double TURRET_GEAR_RATIO =  1.d/((12.d)/(54.d) * (10.d)/(100.d));

  private static final double POSITION_OFFSET = 20.0; //TODO find offset using chinese remainder theorem

  /** Creates a new Turret. */
  public Turret() {
    m_turretMotor = new TalonFX(Constants.TURRET_CAN_ID); //TODO add the motor id constant
    m_thrubore1 =  new CANcoder(Constants.CAN_CODER1);
    m_thrubore2 = new CANcoder(Constants.CAN_CODER2);

    var toApply = new CANcoderConfiguration();
    m_thrubore1.getConfigurator().apply(toApply);
    m_thrubore2.getConfigurator().apply(toApply);

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.Feedback.withSensorToMechanismRatio(TURRET_GEAR_RATIO);
    
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    
    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;
    //setting up motionmagic configs
    MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
    magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROT_PER_SEC; 
    magicConfigs.MotionMagicAcceleration = MAX_ACC_ROT_PER_SEC_SQ; 

    m_turretMotor.getConfigurator().apply(talonFXConfigs);
  
    //go to position, but igure out how to go to position based on limitations
    //compute nearest position clockwise or counterclockwise with + or -, one side og 0 is -, other is +
    //whichever is closer if both follow rules 

    syncRelativeEncoders();
    
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {    
    SmartDashboard.putNumber("turret/goalAngle", m_goal.getDegrees());
    SmartDashboard.putNumber("turret/currentAngle", getPosition().getDegrees());
    SmartDashboard.putNumber("turret/rawMotorAngle", m_turretMotor.getPosition().getValueAsDouble()*360);
    //SmartDashboard.putNumber("turret/chineseRemainderPositionRotations", ChineseRemainder.findAngle(Rotation2d.fromRotations(m_thrubore1.getAbsolutePosition().getValueAsDouble()), 11, Rotation2d.fromRotations((m_thrubore2.getAbsolutePosition().getValueAsDouble()))
    //, 13, 100).getRotations());
  
  }


    public double calculateTurretAngle(double setAngle, double minAngle, double maxAngle, double currentAngle) {
    // Normalize set angle to 0-360
    double normalizedSetAngle = setAngle % 360;
    if (normalizedSetAngle < 0) {
        normalizedSetAngle += 360;
    }
    
    // Find all possible target angles that correspond to the desired position
    // These are: normalizedSetAngle, normalizedSetAngle ± 360, normalizedSetAngle ± 720, etc.
    // But we only need to check nearby rotations since our range is limited
    double[] candidates = {
        normalizedSetAngle - 360,
        normalizedSetAngle,
        normalizedSetAngle + 360
    };
    
    double bestAngle = currentAngle;
    double shortestDistance = Double.MAX_VALUE;
    
    for (double candidate : candidates) {
        // Check if this candidate is within physical limits
        if (candidate >= minAngle && candidate <= maxAngle) {
            double distance = Math.abs(candidate - currentAngle);
            
            if (distance < shortestDistance) {
                shortestDistance = distance;
                bestAngle = candidate;
            }
        }
    }
    
    return bestAngle;
}


  public void set(Rotation2d angle) {
    m_goal = limitRotation(angle);
    m_turretMotor.setControl(new MotionMagicVoltage(m_goal.getRotations()));
  }

//get position of turret
  public Rotation2d getPosition(){
    return Rotation2d.fromRotations(m_turretMotor.getPosition().getValueAsDouble());
  }
    

  public void syncRelativeEncoders(){
    Rotation2d turretAngle = ChineseRemainder.findAngle(Rotation2d.fromRotations(m_thrubore1.getAbsolutePosition().getValueAsDouble()), 11, Rotation2d.fromRotations((m_thrubore2.getAbsolutePosition().getValueAsDouble()))
    , 13, 100);
    m_turretMotor.setPosition(turretAngle.getRotations()-POSITION_OFFSET/360.0);

  }

  private Rotation2d limitRotation(Rotation2d angle){
    return Rotation2d.fromRadians(MathUtil.clamp(angle.getDegrees(), MIN_ROTATION.getDegrees(), MAX_ROTATION.getDegrees()));
    //optimization in here
    //optimal path goes to 

  }
}