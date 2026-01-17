package frc.robot.utilities;

import java.security.Key;

import edu.wpi.first.math.geometry.Rotation2d;

public class ChineseRemainder {
    public static Double FindAngle(Rotation2d RotationsEnc1, int TotalTeeth1, Rotation2d RotationsEnc2, int TotalTeeth2) {
        double RotatedTeeth1 = RotationsEnc1.getRotations() * TotalTeeth1;
        double RotatedTeeth2 = RotationsEnc2.getRotations() * TotalTeeth2;

        return 0.0;
    }
}