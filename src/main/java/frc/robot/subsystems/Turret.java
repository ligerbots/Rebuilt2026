// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//look for motor ratios
package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utilities.ChineseRemainder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class Turret extends SubsystemBase {
    
    private static final Translation2d TURRET_OFFSET = new Translation2d(Units.inchesToMeters(8.33),  Units.inchesToMeters(-4.36)); // TODO: Check offset hasn't changed
    private static final Rotation2d TURRET_ANGLE_TOLERANCE = Rotation2d.fromDegrees(2.0); // TODO: Tune this value
    
    private Rotation2d m_goal = Rotation2d.kZero;
    
    private final TalonFX m_turretMotor;
    private final CANcoder m_thruboreSmall; 
    private final CANcoder m_thruboreLarge; 
    
    private static final double SUPPLY_CURRENT_LIMIT = 60;
    private static final double STATOR_CURRENT_LIMIT = 40;

    private static final int ENCODER_SMALL_TOOTH_COUNT = 11;
    private static final int ENCODER_LARGE_TOOTH_COUNT = 13;
    private static final int TURRET_TOOTH_COUNT = 100;
    private static final double TURRET_GEAR_RATIO =  54.0 / 12.0 * TURRET_TOOTH_COUNT / 10.0;
    
    private static final double K_P = 2.0; 
    private static final double MAX_VEL_ROT_PER_SEC = 4.0 * TURRET_GEAR_RATIO;
    private static final double MAX_ACC_ROT_PER_SEC_SQ = 40.0 * TURRET_GEAR_RATIO;
    private static final Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(170.0);
    private static final Rotation2d MIN_ROTATION = Rotation2d.fromDegrees(-170.0);
    
    private static final Rotation2d CRT_POSITION_OFFSET = Rotation2d.fromDegrees(322.3);
    
    /** Creates a new Turret. */
    public Turret() {
        m_turretMotor = new TalonFX(Constants.TURRET_CAN_ID);
        m_thruboreSmall =  new CANcoder(Constants.TURRET_SMALL_CANCODER_ID);
        m_thruboreLarge = new CANcoder(Constants.TURRET_LARGE_CANCODER_ID);
        
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        m_thruboreSmall.getConfigurator().apply(cancoderConfig);
        m_thruboreLarge.getConfigurator().apply(cancoderConfig);
        
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
                
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;
        slot0configs.kI = 0.0;
        slot0configs.kD = 0.0;
        
        // setting up motionmagic configs
        MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROT_PER_SEC; 
        magicConfigs.MotionMagicAcceleration = MAX_ACC_ROT_PER_SEC_SQ; 
        
        m_turretMotor.getConfigurator().apply(talonFXConfigs);
        
        // double zeroPos = 0.0;
        double zeroPos = getCRTAngle().getRotations() * TURRET_GEAR_RATIO;
        m_turretMotor.setPosition(zeroPos);
    }
    
    // This method will be called once per scheduler run
    @Override
    public void periodic() {    
        SmartDashboard.putNumber("turret/goalAngle", m_goal.getDegrees());
        SmartDashboard.putNumber("turret/currentAngle", getAngle().getDegrees());

        // Values for testing and tuning
        SmartDashboard.putNumber("turret/crtAngleRaw",getCRTAngleRaw().getDegrees());   
        SmartDashboard.putNumber("turret/crtAngle",getCRTAngle().getDegrees());   
        SmartDashboard.putNumber("turret/motorVel", m_turretMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("turret/absEncoder2", m_thruboreLarge.getAbsolutePosition().getValueAsDouble()*360);
        SmartDashboard.putNumber("turret/absEncoder1", m_thruboreSmall.getAbsolutePosition().getValueAsDouble()*360);
    }
    
    private double calculateTurretAngle(double setAngle, double minAngle, double maxAngle, double currentAngle) {
        // Normalize set angle to 0-360
        double normalizedSetAngle = setAngle % 360.0;
        if (normalizedSetAngle < 0) {
            normalizedSetAngle += 360.0;
        }
        
        // Find all possible target angles that correspond to the desired position
        // These are: normalizedSetAngle, normalizedSetAngle ± 360, normalizedSetAngle ± 720, etc.
        // But we only need to check nearby rotations since our range is limited
        double[] candidates = {
            normalizedSetAngle - 360.0,
            normalizedSetAngle,
            normalizedSetAngle + 360.0
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
       
    public void setAngle(Rotation2d angle) {
        m_goal = limitRotation(angle);
        m_turretMotor.setControl(new MotionMagicVoltage(m_goal.getRotations() * TURRET_GEAR_RATIO));
    }
    
    // get angle of turret
    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(m_turretMotor.getPosition().getValueAsDouble() / TURRET_GEAR_RATIO);
    }
        
    private Rotation2d getCRTAngleRaw(){
        return ChineseRemainder.findAngle(
                Rotation2d.fromRotations(m_thruboreSmall.getAbsolutePosition().getValueAsDouble()), ENCODER_SMALL_TOOTH_COUNT,
                Rotation2d.fromRotations(m_thruboreLarge.getAbsolutePosition().getValueAsDouble()), ENCODER_LARGE_TOOTH_COUNT,
                TURRET_TOOTH_COUNT);
    }

    private Rotation2d getCRTAngle() {
        return getCRTAngleRaw().minus(CRT_POSITION_OFFSET);
    }

    private Rotation2d limitRotation(Rotation2d angle){
        return Rotation2d.fromDegrees(MathUtil.clamp(angle.getDegrees(), MIN_ROTATION.getDegrees(), MAX_ROTATION.getDegrees()));
        //optimization in here
        //optimal path goes to 
    }
    
    public boolean isTurretAtTargetForHub(Pose2d robotPose) {
        return Math.abs(getAngle().minus(getAngleToHub(robotPose)).getRadians()) < TURRET_ANGLE_TOLERANCE.getRadians(); 
    }
    
    private static Translation2d getTurretPositionForRobotPose(Pose2d robotPose) {
        // Pose2d robotPose = m_drivetrain.getState().Pose;
        return Turret.TURRET_OFFSET.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
    }
    
    public static Rotation2d getAngleToHub(Pose2d robotPose) {
        Rotation2d overallAngle = getTurretPositionForRobotPose(robotPose).minus(FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE)).getAngle();
        return overallAngle.plus(robotPose.getRotation());
    }
    
    public static double getDistanceToHub(Pose2d robotPose) {
        return getTurretPositionForRobotPose(robotPose).getDistance(FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE));
    }
}