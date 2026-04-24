// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//look for motor ratios
package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.utilities.ChineseRemainder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.RobotLog;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class Turret extends SubsystemBase {
    // public for the Shoot command - not the greatest, but a pain otherwise
    public static final Translation2d TURRET_OFFSET = new Translation2d(Units.inchesToMeters(-2.5626),  Units.inchesToMeters(-4.875));
    private static final double TURRET_HEADING_OFFSET_DEG = 180.0;
    private static final double ANGLE_TOLERANCE_DEG = 10.0;

    private static final double DEADZONE_TOLERANCE_DEG = 2.0; 
    private static final double SIDE_FLIP_TOLERANCE_DEG = 10.0; 
    
    private double m_goalDeg = 0.0; // angle limited to our constraints
    private double m_shootAngle = 0.0; // raw shoot angle (actual angle we want to shoot)
    
    private final TalonFX m_turretMotor;
    private final CANcoder m_thruboreSmall; 
    private final CANcoder m_thruboreLarge; 
    
    private static final Current SUPPLY_CURRENT_LIMIT = Amps.of(20);
    private static final Current STATOR_CURRENT_LIMIT = Amps.of(60);

    // Manual turret angle adjustment (additive)
    private double m_turretFudgeDegrees = 0;
    private static final double TURRET_FUDGE_INCREMENT_DEGREES = 1;

    private static final int ENCODER_SMALL_TOOTH_COUNT = 11;
    private static final int ENCODER_LARGE_TOOTH_COUNT = 13;
    private static final double ENCODER_SMALL_OFFSET_ROTATIONS = -0.501;
    private static final double ENCODER_LARGE_OFFSET_ROTATIONS = -0.272;
    private static final int TURRET_TOOTH_COUNT = 100;
    private static final double TURRET_GEAR_RATIO =  54.0 / 12.0 * TURRET_TOOTH_COUNT / 10.0;
    
    private static final double K_P = 1.9;    // tuned 4/10
    private static final double K_D = 0.0;
    private static final double K_I = 0.0;
    private static final double K_S = 0.1;    // not sure this helps? but does not hurt
    private static final double K_V = 0.1;    // roughly correct 3/25 (hard to measure)
    private static final double K_A = 0.0;

    private static final double MAX_VEL_ROT_PER_SEC = 10.0 * TURRET_GEAR_RATIO;
    private static final double MAX_ACC_ROT_PER_SEC_SQ = 40.0 * TURRET_GEAR_RATIO;
    //
    // 2/14 - Slowed down for testing, until chain working properly
    // private static final double MAX_VEL_ROT_PER_SEC = 2.0 * TURRET_GEAR_RATIO;
    // private static final double MAX_ACC_ROT_PER_SEC_SQ = 10.0 * TURRET_GEAR_RATIO;

    private static final double MAX_ROTATION_DEG = 165.0;
    private static final double MIN_ROTATION_DEG = -190.0; // -220 is max
    private static final double MID_LINE_DEGREES = (MAX_ROTATION_DEG + MIN_ROTATION_DEG) / 2.0;
    
    // this is just the middle point of the full CRT range
    // include "1.0 *" to make sure it does floating point arithmetic
    private static final Rotation2d CRT_POSITION_OFFSET = 
            Rotation2d.fromRotations(1.0 * ENCODER_SMALL_TOOTH_COUNT * ENCODER_LARGE_TOOTH_COUNT / TURRET_TOOTH_COUNT / 2.0);
    
    private Field2d m_field;

    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0).withEnableFOC(true);

    /** Creates a new Turret. */
    public Turret(Field2d field) {
        // Field is used for plotting the heading
        m_field = field;

        m_turretMotor = new TalonFX(Constants.TURRET_CAN_ID);
        m_thruboreSmall =  new CANcoder(Constants.TURRET_SMALL_CANCODER_ID);
        m_thruboreLarge = new CANcoder(Constants.TURRET_LARGE_CANCODER_ID);
        
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        m_thruboreSmall.getConfigurator().apply(cancoderConfig);
        m_thruboreLarge.getConfigurator().apply(cancoderConfig);
        
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
                   
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        slot0configs.kP = K_P;
        slot0configs.kI = K_I;
        slot0configs.kD = K_D;
        slot0configs.kV = K_V;
        slot0configs.kS = K_S;
        slot0configs.kA = K_A;

        // setting up motionmagic configs
        MotionMagicConfigs magicConfigs = talonFXConfigs.MotionMagic;
        magicConfigs.MotionMagicCruiseVelocity = MAX_VEL_ROT_PER_SEC; 
        magicConfigs.MotionMagicAcceleration = MAX_ACC_ROT_PER_SEC_SQ; 

         CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        talonFXConfigs.withCurrentLimits(currentLimits);
        
        m_turretMotor.getConfigurator().apply(talonFXConfigs);
        
        // double zeroPos = 0.0;
        double zeroPos = getCRTAngle().getRotations() * TURRET_GEAR_RATIO;
        m_turretMotor.setPosition(zeroPos);

        // NOTE: this must be done after zeroing the turret
        // We set the updates from the ThroughBores to be pretty slow
        if (Constants.OPTIMIZE_CAN) {
            optimizeCAN();
        }
    }
    
    private void optimizeCAN() {
        // For the turret, we want the position every loop
        m_turretMotor.getPosition().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        
        // for debug?
        m_turretMotor.getMotorVoltage().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_turretMotor.getVelocity().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);
        m_turretMotor.getAcceleration().setUpdateFrequency(Constants.ROBOT_FREQUENCY_HZ);

        m_turretMotor.optimizeBusUtilization();

        // for the throughbores, we don't need frequent values after init
        m_thruboreSmall.optimizeBusUtilization();
        m_thruboreLarge.optimizeBusUtilization();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {    
        double goal = getGoalDeg();
        double currentAngle = getAngle().getDegrees();

        // Driver-facing status
        SmartDashboard.putNumber("turret/currentAngle", currentAngle);
        SmartDashboard.putNumber("turret/fudgeAngle", m_turretFudgeDegrees);

        // Tracking state
        RobotLog.log("turret/angleError", goal - currentAngle);

        // Motor electrical/motion data
        RobotLog.log("turret/voltage", m_turretMotor.getMotorVoltage().getValueAsDouble());
        RobotLog.log("turret/velocityRPS", m_turretMotor.getVelocity().getValueAsDouble());
        RobotLog.log("turret/accelRPS2", m_turretMotor.getAcceleration().getValueAsDouble());

        // Values for testing and tuning
        // SmartDashboard.putNumber("turret/crtAngleRaw",getCRTAngleRaw().getDegrees());   
        // SmartDashboard.putNumber("turret/crtAngle",getCRTAngle().getDegrees());
        
        // USE ME FOR TUNING ABSOLUTE ENCODER OFFSETS ONLY:
        // ChineseRemainder.smartDashboardLogABSOffsets(ENCODER_SMALL_TOOTH_COUNT, ENCODER_LARGE_TOOTH_COUNT, 
        //         m_thruboreSmall.getAbsolutePosition().getValueAsDouble(),
        //         m_thruboreLarge.getAbsolutePosition().getValueAsDouble());
    }

    public void increaseTurretFudge() {
        m_turretFudgeDegrees += TURRET_FUDGE_INCREMENT_DEGREES;
    }

    public void decreaseTurretFudge() {
        m_turretFudgeDegrees -= TURRET_FUDGE_INCREMENT_DEGREES;
    }
    
    public void setAngle(Rotation2d angle) {
        // WARNING: this code is intended for a travel range with a dead zone
        // If we manage to get >360 travel, this need to be reworked

        // remember the requested angle, but in turret coordinates
        //   and mapped onto the range of travel
        m_shootAngle = degreesModulus(angle.getDegrees() + m_turretFudgeDegrees - TURRET_HEADING_OFFSET_DEG);
    
        // for now, just limit angle to not go into the dead zone
        m_goalDeg = limitRotationDeg(m_shootAngle);

        m_positionControl.Position = m_goalDeg/360.0 * TURRET_GEAR_RATIO;
        m_turretMotor.setControl(m_positionControl);

        // update these log items right away
        RobotLog.log("turret/shootAngle", m_shootAngle + TURRET_HEADING_OFFSET_DEG);
        RobotLog.log("turret/goalAngle", m_goalDeg + TURRET_HEADING_OFFSET_DEG);
    }
    
    // get angle of turret
    public Rotation2d getAngle(){
        double rot = m_turretMotor.getPosition().getValueAsDouble() / TURRET_GEAR_RATIO + TURRET_HEADING_OFFSET_DEG / 360.0;
        return Rotation2d.fromRotations(rot);
    }
       
    // private so we don't need to create a Rotation2d
    private double getGoalDeg(){
        return m_goalDeg + TURRET_HEADING_OFFSET_DEG;
    }

    // private double getShootAngleDeg() {
    //     return m_shootAngle + TURRET_HEADING_OFFSET_DEG;
    // }

    public boolean inDeadZone() {
        // wrap the error to +/- 180 - needed when the goal is ~0
        double goalErrDeg = MathUtil.inputModulus(m_shootAngle - m_goalDeg, -180.0, 180.0);
        double positionErrDeg = MathUtil.inputModulus(getAngle().getDegrees() - getGoalDeg(), -180.0, 180.0);

        // return Math.abs(goalErrDeg) >= DEADZONE_TOLERANCE_DEG;
        return Math.abs(goalErrDeg) >= DEADZONE_TOLERANCE_DEG || Math.abs(positionErrDeg) >= SIDE_FLIP_TOLERANCE_DEG;
    }

    public boolean isOnTarget() {
        // wrap the error to +/- 180 - needed when the goal is ~0
        double errorDeg = MathUtil.inputModulus(
                getAngle().getDegrees() - getGoalDeg(),
                -180.0,
                180.0);
        return Math.abs(errorDeg) < ANGLE_TOLERANCE_DEG; 
    }
    
    private Rotation2d getCRTAngleRaw(){
        return ChineseRemainder.findAngle(
                m_thruboreSmall.getAbsolutePosition().getValueAsDouble() + ENCODER_SMALL_OFFSET_ROTATIONS,
                ENCODER_SMALL_TOOTH_COUNT,
                m_thruboreLarge.getAbsolutePosition().getValueAsDouble() + ENCODER_LARGE_OFFSET_ROTATIONS,
                ENCODER_LARGE_TOOTH_COUNT,
                TURRET_TOOTH_COUNT);
    }

    private Rotation2d getCRTAngle() {
        return getCRTAngleRaw().minus(CRT_POSITION_OFFSET);
    }

    private double limitRotationDeg(double angleDeg) {
        return MathUtil.clamp(angleDeg, MIN_ROTATION_DEG, MAX_ROTATION_DEG);
    }
    
    /**
     * Map angle (in degrees) to -180 ==> 180 but centered on the midpoint of travel
     * Analogous to MathUtils.angleModulus()
     * @param angleDeg  angle in degrees
     * @return angle    same angle but mapped to -180 ==> 180
     */
    private double degreesModulus(double angleDeg) {
        while (angleDeg >= (MID_LINE_DEGREES + 180.0)) angleDeg -= 360.0;
        while (angleDeg < (MID_LINE_DEGREES - 180.0)) angleDeg += 360.0;
        return angleDeg;
    }

    // compute the optimum goal angle
    // this assumes the Turret can turn >360 degrees
    // private double optimizeGoal(double setAngleDeg) {
    //     // Normalize target angle to -180 -> 180
    //     setAngleDeg = degreesModulus(setAngleDeg);
        
    //     // Find all possible target angles that correspond to the desired position
    //     // These are: normalizedSetAngle, normalizedSetAngle ± 360, normalizedSetAngle ± 720, etc.
    //     // But we only need to check nearby rotations since our range is limited
    //     double[] candidates = {
    //         setAngleDeg - 360.0,
    //         setAngleDeg,
    //         setAngleDeg + 360.0
    //     };
        
    //     double currentAngle = getAngle().getDegrees();

    //     double bestAngle = currentAngle;
    //     double shortestDistance = Double.MAX_VALUE;
    //     for (double candidate : candidates) {
    //         // Check if this candidate is within physical limits
    //         if (candidate >= MIN_ROTATION_DEG && candidate <= MAX_ROTATION_DEG) {
    //             double distance = Math.abs(candidate - currentAngle);
                
    //             if (distance < shortestDistance) {
    //                 shortestDistance = distance;
    //                 bestAngle = candidate;
    //             }
    //         }
    //     }
        
    //     // System.out.println("Best Angle: " + bestAngle);

    //     return bestAngle;
    // }

    public static Translation2d getTranslationToGoal(Pose2d robotPose, Translation2d target) {
        // compute robot to target, in field coordinates
        Translation2d robotToTargetField = target.minus(robotPose.getTranslation());
        // System.out.println("r2t_f " + robotPose.getTranslation() + " --> " + target + " = " + robotToTargetField);

        // rotate that into robot coordinates
        Translation2d robotToTargetRobot = robotToTargetField.rotateBy(robotPose.getRotation().unaryMinus());
        // System.out.println("r2t_r " + robotPose.getRotation().getDegrees() + " = " + robotToTargetRobot);

        // subtract off the turret offset
        Translation2d turretToTarget = robotToTargetRobot.minus(TURRET_OFFSET);
        // System.out.println("tur2t " + TURRET_OFFSET + " = " + turretToTarget);

        // done
        return turretToTarget;
    }

    public void plotShotVectors(Pose2d robotPose, Translation2d targetVector, Translation2d robotMotion, Translation2d centripetalMotion) {
        try {
            if (robotPose == null || targetVector == null) {
                clearShotVisualization();
                return;
            }

            Translation2d turretLoc = getTurretFieldPosition(robotPose);

            Rotation2d turretHeadingRobot = targetVector.getAngle();
            Rotation2d turretHeadingField = turretHeadingRobot.rotateBy(robotPose.getRotation());

            Translation2d robotEndLoc = turretLoc.plus(new Translation2d(targetVector.getNorm(), turretHeadingField));

            Translation2d robotVecEndLoc = robotEndLoc.plus(robotMotion);
            Translation2d centripetalEndLoc = robotVecEndLoc.plus(centripetalMotion);

            m_field.getObject("turretHeading").setPoses(
                    new Pose2d(turretLoc, turretHeadingField),
                    new Pose2d(robotEndLoc, turretHeadingField)
            );
            m_field.getObject("shotTarget").setPose(new Pose2d(robotEndLoc, Rotation2d.kZero));

            m_field.getObject("robotHeading").setPoses(
                    new Pose2d(robotEndLoc, robotMotion.getAngle()),
                    new Pose2d(robotVecEndLoc, robotMotion.getAngle()) 
            );
            m_field.getObject("centripetalHeading").setPoses(
                    new Pose2d(robotVecEndLoc, centripetalMotion.getAngle()),
                    new Pose2d(centripetalEndLoc, centripetalMotion.getAngle())  
            );
        } catch (Exception e) {
            // bad! log this and keep going
            DriverStation.reportError("Exception plotting shot vectors", e.getStackTrace());
        }
    }

    public void clearShotVisualization() {
        m_field.getObject("turretHeading").setPoses();
        m_field.getObject("robotHeading").setPoses();
        m_field.getObject("centripetalHeading").setPoses();
        m_field.getObject("shotTarget").setPoses();
    }

    public static Translation2d getTurretFieldPosition(Pose2d robotPose) {
        return TURRET_OFFSET.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
    }
    
    // private static void runTests() {
    //     Translation2d position = FieldConstants.HUB_POSITION_BLUE.minus(new Translation2d(1.0, 0.5));

    //     for (double angle = 0; angle < 360; angle += 45.0) {
    //         System.out.println("Angle = " + angle);

    //         Pose2d robot = new Pose2d(position, Rotation2d.fromDegrees(angle));

    //         Translation2d t1 = getTranslationToGoalOld(robot, FieldConstants.HUB_POSITION_BLUE);
    //         System.out.println("getT2G: " + t1);
    //         Translation2d t2 = getTranslationToGoal(robot, FieldConstants.HUB_POSITION_BLUE);
    //         System.out.println("getT2TPaul: " + t2);
    //     }
    // }

    // public static void main(String[] args) {
    //     runTests();
    // }
}
