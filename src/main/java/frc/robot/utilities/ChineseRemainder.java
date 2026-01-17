package frc.robot.utilities;

import java.security.Key;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChineseRemainder {
    // public static Double FindAngle(Rotation2d RotationsEnc1, int TotalTeeth1, Rotation2d RotationsEnc2, int TotalTeeth2) {
    public static Double FindAngle(Rotation2d RotationsEnc1, int TotalTeeth1, Rotation2d RotationsEnc2, int TotalTeeth2) {
        double RotatedTeeth1 = RotationsEnc1.getRotations() * TotalTeeth1;
        double RotatedTeeth2 = RotationsEnc2.getRotations() * TotalTeeth2;

         /*
         * N = i*k1 + n1
         * N = j*k2 + n2
         */

        double bestResultDifference = 1e12; // closer to 0 = more accurate. Both calculations are the most similar
        double currentResult1 = -1;

        for (int i = 0; i < TotalTeeth2; i++) { // finding i
            double test = i * TotalTeeth1 + RotatedTeeth1;
            double remainderCheck = test % TotalTeeth2;

            double resultDif = Math.abs(remainderCheck - RotatedTeeth2);
            if (resultDif < bestResultDifference) {
                bestResultDifference = resultDif;
                currentResult1 = test;
            }
        }

        
        bestResultDifference = 1e12;
        double currentResult2 = -1;

        for (int i = 0; i < TotalTeeth1; i++) { // finding j
            double test = i * TotalTeeth2 + RotatedTeeth2;
            double remainderCheck = test % TotalTeeth1;

            double resultDif = Math.abs(remainderCheck - RotatedTeeth1);
            if (resultDif < bestResultDifference) {
                bestResultDifference = resultDif;
                currentResult2 = test;
            }
        }

        System.out.println((currentResult1 + currentResult2)/2);

        return (currentResult1 + currentResult2)/2;
    }
}