package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static final double FIELD_LENGTH = FlippingUtil.fieldSizeX;
    public static final double FIELD_WIDTH = FlippingUtil.fieldSizeY;  
    public static final double SHOOT_HUB_LINE_BLUE = Units.inchesToMeters(158); // distance from the wall to the line where we want to shoot from
    public static final boolean ENABLE_UPDATED_SHOT_LOGIC = true;
    public static final boolean ENABLE_OPPOSITE_ZONE_SHOT = ENABLE_UPDATED_SHOT_LOGIC;
    public static final boolean ENABLE_DYNAMIC_PASS_TARGETING = ENABLE_UPDATED_SHOT_LOGIC;
    public static final boolean ENABLE_PASS_SIDE_LATCH = ENABLE_UPDATED_SHOT_LOGIC;
    public static final boolean USE_TURRET_POSITION_FOR_SHOT_SELECTION = ENABLE_UPDATED_SHOT_LOGIC;

    public static final double OPPOSITE_ALLIANCE_ZONE_START_X_BLUE = Units.inchesToMeters(500);//taken from far side of opposite bump 
    public static final double SIDE_WALL_TARGET_OFFSET_BLUE = Units.inchesToMeters(24.0); //TODO just a guess for now, needs to get tuned alongside LUT
    public static final double SIDE_WALL_TARGET_LINE_MAX_DISTANCE_BLUE = Units.inchesToMeters(135.0);// this is the number used to translate robot to wall horizontal distance to vertical shot distance
    public static final double PASS_LATCH_BASE_Y_TOLERANCE_BLUE = Units.inchesToMeters(18.0); //TODO these are based on vibes need to tune
    public static final double PASS_LATCH_MAX_YAW_DEGREES = 22.0; //TODO these are based on vibes need to tune 
    
    public static final double OPPOSITE_ALLIANCE_ZONE_FAR_TARGET_DISTANCE = Units.inchesToMeters(445.0);//Far is relative to driverstation. Number is taken from cad as the distance to the end of the bump
    public static final double OPPOSITE_ALLIANCE_ZONE_CLOSE_TARGET_DISTANCE = Units.inchesToMeters(290);//Far is relative to driverstation. Number is roughly distance from wall to 4th side wall support strut

    
    public static final Translation2d HUB_POSITION_BLUE = new Translation2d(Units.inchesToMeters(182.11),Units.inchesToMeters(158.84));
    public static final Translation2d PASSING_TARGET_LEFT_BLUE = new Translation2d(Units.inchesToMeters(91.6), Units.inchesToMeters(66));
    public static final Translation2d PASSING_TARGET_RIGHT_BLUE = new Translation2d(Units.inchesToMeters(91.6), Units.inchesToMeters(250));
    
    public static final Translation2d OPPOSITE_ZONE_TARGET_LINE_LEFT_NEAR_BLUE =
        new Translation2d(OPPOSITE_ALLIANCE_ZONE_CLOSE_TARGET_DISTANCE, SIDE_WALL_TARGET_OFFSET_BLUE);
    
    public static final Translation2d OPPOSITE_ZONE_TARGET_LINE_LEFT_FAR_BLUE =
        new Translation2d(OPPOSITE_ALLIANCE_ZONE_FAR_TARGET_DISTANCE, SIDE_WALL_TARGET_OFFSET_BLUE);
    
    public static final Translation2d OPPOSITE_ZONE_TARGET_LINE_RIGHT_NEAR_BLUE =
        new Translation2d(OPPOSITE_ALLIANCE_ZONE_CLOSE_TARGET_DISTANCE, FIELD_WIDTH - SIDE_WALL_TARGET_OFFSET_BLUE);
    
    public static final Translation2d OPPOSITE_ZONE_TARGET_LINE_RIGHT_FAR_BLUE =
        new Translation2d(OPPOSITE_ALLIANCE_ZONE_FAR_TARGET_DISTANCE, FIELD_WIDTH - SIDE_WALL_TARGET_OFFSET_BLUE);
 
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    // The "flip" routines will transpose a Pose/Translation from Field to Blue, or vice versa.
    // For 2026, this is *actually* a rotation around the center.
    // When "flipping" a Pose, the heading angle is rotated so that 
    //   it looks correct on the new side (looks the same from the driver's station).

    public static Pose2d flipPose(Pose2d pose) {
        // flip pose when red
        if (isRedAlliance()) {
            return FlippingUtil.flipFieldPose(pose);
        }
        // Blue or we don't know; return the original pose
        return pose;
    }

    public static Translation2d flipTranslation(Translation2d position) {
        // flip when red
        if (isRedAlliance()) {
            return FlippingUtil.flipFieldPosition(position);
        }
        // Blue or we don't know; return the original position
        return position;
    }

    // NOTE: the "mirror" routines will mirror a Pose/Translation along the specified axis
    // They do NOT check Red vs Blue. They always do the mirror operation.
    // Careful: you probably do NOT want this. The "flip" routines are almost always what is needed.

    // Feb 2026: Commented out for now. Not used and dangerous if you are not aware of the differences.
    
    // public static Translation2d mirrorTranslationY(Translation2d translation) {
    //     return new Translation2d(translation.getX(), FlippingUtil.fieldSizeY - translation.getY());
    // }

    // public static Translation2d mirrorTranslationX(Translation2d translation) {
    //     return new Translation2d(FlippingUtil.fieldSizeX - translation.getX(), translation.getY());
    // }

    // public static Pose2d mirrorPoseY(Pose2d pose) {
    //     return new Pose2d(mirrorTranslationY(pose.getTranslation()), pose.getRotation().unaryMinus());
    // }
}
