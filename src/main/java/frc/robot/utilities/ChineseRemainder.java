package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;

public class ChineseRemainder {

    public static Double FindAngle(Rotation2d RotationsEnc1, int TotalTeeth1, Rotation2d RotationsEnc2, int TotalTeeth2) {
        double RotatedTeeth1 = RotationsEnc1.getRotations() * TotalTeeth1;
        double RotatedTeeth2 = RotationsEnc2.getRotations() * TotalTeeth2;

        double bestN = -1;
        double bestError = Double.MAX_VALUE;

        // Use LCM or max of the two for reasonable search range
        int searchLimit = Math.max(TotalTeeth1, TotalTeeth2);
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
        return bestN;
    }

    public static void runTests() {
        // AI generated tests for Chinese Remainder Theorem implementation. Good enough for sanity checking.
        // All tests use 11-tooth and 13-tooth gears (coprime for unambiguous solutions)

        System.out.println("=== Chinese Remainder Theorem Tests ===\n");

        // Test 1: Both encoders agree closely
        System.out.println("Test 1: Both encoders at 90°");
        double result1 = ChineseRemainder.FindAngle(
                Rotation2d.fromDegrees(90), 11,
                Rotation2d.fromDegrees(90), 13);
        System.out.println("Result: " + result1);
        System.out.println("Expected: ~2.75-3.25 teeth");
        System.out.println("(Enc1: 2.75 teeth, Enc2: 3.25 teeth)\n");

        // Test 2: One full rotation on both encoders
        System.out.println("Test 2: Full rotation on both encoders");
        double result2 = ChineseRemainder.FindAngle(
                Rotation2d.fromDegrees(360), 11,
                Rotation2d.fromDegrees(360), 13);
        System.out.println("Result: " + result2);
        System.out.println("Expected: ~0.0 teeth (both wrap to zero)");
        System.out.println("(Enc1: 11.0→0 mod 11, Enc2: 13.0→0 mod 13)\n");

        // Test 3: Half rotation
        System.out.println("Test 3: Half rotation (180°)");
        double result3 = ChineseRemainder.FindAngle(
                Rotation2d.fromDegrees(180), 11,
                Rotation2d.fromDegrees(180), 13);
        System.out.println("Result: " + result3);
        System.out.println("Expected: ~5.5-6.5 teeth");
        System.out.println("(Enc1: 5.5 teeth, Enc2: 6.5 teeth)\n");

        // Test 4: Small angles
        System.out.println("Test 4: Small angles (5°)");
        double result4 = ChineseRemainder.FindAngle(
                Rotation2d.fromDegrees(5), 11,
                Rotation2d.fromDegrees(5), 13);
        System.out.println("Result: " + result4);
        System.out.println("Expected: ~0.15-0.18 teeth");
        System.out.println("(Enc1: 0.153 teeth, Enc2: 0.181 teeth)\n");

        // Test 5: Larger rotation with disagreement
        System.out.println("Test 5: Disagreeing encoders");
        double result5 = ChineseRemainder.FindAngle(
                Rotation2d.fromDegrees(120), 11,
                Rotation2d.fromDegrees(130), 13);
        System.out.println("Result: " + result5);
        System.out.println("Expected: ~3.5-4.0 teeth");
        System.out.println("(Enc1: 3.67 teeth, Enc2: 4.69 teeth)\n");

        // Test 6: Near-full rotation
        System.out.println("Test 6: Near-full rotation");
        double result6 = ChineseRemainder.FindAngle(
                Rotation2d.fromDegrees(350), 11,
                Rotation2d.fromDegrees(355), 13);
        System.out.println("Result: " + result6);
        System.out.println("Expected: ~0.0 teeth (wraps around)");
        System.out.println("(Enc1: 10.69 teeth ≈ 0.31 from full, Enc2: 12.82 teeth ≈ 0.18 from full)\n");

        System.out.println("=== Tests Complete ===");
    }
}