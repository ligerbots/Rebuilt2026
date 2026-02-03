package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static final double FIELD_LENGTH = FlippingUtil.fieldSizeX;
    public static final double FIELD_WIDTH = FlippingUtil.fieldSizeY;  

    public static final Translation2d HUB_POSITION_BLUE = new Translation2d(158.32,181.56);
 
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

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

    // Remove? Shouldn't be needed
    public static Translation2d mirrorTranslation(Translation2d translation) {
        return new Translation2d(translation.getX(), FlippingUtil.fieldSizeY - translation.getY());
      }

    // Remove? Shouldn't be needed
    public static Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(mirrorTranslation(pose.getTranslation()), pose.getRotation().unaryMinus());
    }
}