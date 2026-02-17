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

    public static final Translation2d HUB_POSITION_BLUE = new Translation2d(Units.inchesToMeters(182.11),Units.inchesToMeters(158.84));
    public static final Translation2d PASSING_TARGET_UPPER_BLUE = new Translation2d(Units.inchesToMeters(91.6), Units.inchesToMeters(238));
    public static final Translation2d PASSING_TARGET_LOWER_BLUE = new Translation2d(Units.inchesToMeters(91.6), Units.inchesToMeters(79.5));
 
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