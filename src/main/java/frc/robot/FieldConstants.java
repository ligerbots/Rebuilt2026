package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static final Translation2d TURRET_OFFSET = flipTranslation(new Translation2d(0.0, 0.0)); // TODO: Figure out actual offset
    public static final Translation2d HUB_POSITION = flipTranslation(new Translation2d(0.0,0.0)); // TODO: figure out actual hub position on field
 
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
}