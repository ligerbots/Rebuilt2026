package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;

public class ChineseRemainder {

    public static Rotation2d FindAngle(Rotation2d RotationsEnc1, int TotalTeeth1, Rotation2d RotationsEnc2, int TotalTeeth2, int BigGearTeeth) {
        double RotatedTeeth1 = RotationsEnc1.getRotations() * TotalTeeth1;
        double RotatedTeeth2 = RotationsEnc2.getRotations() * TotalTeeth2;

        double bestN = -1;
        double bestError = Double.MAX_VALUE;

        // Use LCM or max of the two for reasonable search range
        int searchLimit = BigGearTeeth;
        double stepSize = 0.01;

        for (double N = 0; N < searchLimit; N += stepSize) {
            double error1 = Math.abs((N % TotalTeeth1) - (RotatedTeeth1 % TotalTeeth1));
            double error2 = Math.abs((N % TotalTeeth2) - (RotatedTeeth2 % TotalTeeth2));

            error1 = Math.min(error1, TotalTeeth1 - error1);
            error2 = Math.min(error2, TotalTeeth2 - error2);

            double totalError = error1 + error2;

            if (totalError < bestError) {
                bestError = totalError;
                bestN = N;
            }
        }

        // System.out.println(bestN);
        return Rotation2d.fromRotations(bestN/BigGearTeeth);
    }

    public static void runTests() {
        // AI generated tests for Chinese Remainder Theorem implementation. Good enough for sanity checking.
        // All tests use 11-tooth and 13-tooth gears (coprime for unambiguous solutions)
        testOverallAngle(Rotation2d.fromDegrees(10.0), 100, 11, 13);
        testOverallAngle(Rotation2d.fromDegrees(45.0), 100, 11, 13);
        testOverallAngle(Rotation2d.fromDegrees(90.0), 100, 11, 13);
        testOverallAngle(Rotation2d.fromDegrees(180.0), 100, 11, 13);
        testOverallAngle(Rotation2d.fromDegrees(270.0), 100, 11, 13);

    }

    // Main gear 100 teeth
    public static void testOverallAngle(Rotation2d bigNAngle, int bigNTeeth, int gear1teeth, int gear2teeth) {
        double gear1remainder = (bigNAngle.getRotations() * bigNTeeth) % gear1teeth;
        double gear2remainder = (bigNAngle.getRotations() * bigNTeeth) % gear2teeth;

        Rotation2d teeth = ChineseRemainder.FindAngle(
                Rotation2d.fromRotations(gear1remainder / gear1teeth), gear1teeth,
                Rotation2d.fromRotations(gear2remainder / gear2teeth), gear2teeth, bigNTeeth);

        System.out.println("Result: " + teeth.getDegrees() + " degrees");
        System.out.println("Expected:" + bigNAngle.getDegrees() + "degrees");
    }
}