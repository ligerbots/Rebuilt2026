package frc.robot.utilities;

import java.util.Random;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Implements Chinese Remainder Theorem to determine absolute encoder positions from two relative encoders
 * with different gear ratios. Creates high-resolution absolute positioning by finding the unique solution
 * that satisfies both encoder readings simultaneously.
 */
public class ChineseRemainder {

    /**
     * Finds absolute angle of a large gear using two encoder readings with different gear ratios.
     * Uses brute-force search to find the position that best satisfies both encoder constraints.
     * 
     * @param rotationsEnc1 First encoder reading (in rotations)
     * @param totalTeeth1 Teeth first encoder sees per big gear revolution
     * @param rotationsEnc2 Second encoder reading (in rotations)  
     * @param totalTeeth2 Teeth second encoder sees per big gear revolution
     * @param bigGearTeeth Number of teeth on the large gear
     * @return Absolute angle of the large gear
     */
    public static Rotation2d findAngle(double rotationsEnc1, int totalTeeth1, double rotationsEnc2, int totalTeeth2, int bigGearTeeth) {
        // Convert encoder rotations to teeth count for easier calculation

        double rotatedTeeth1 = wrapRotation(rotationsEnc1) * totalTeeth1;
        double rotatedTeeth2 = wrapRotation(rotationsEnc2) * totalTeeth2;

        double bestN1 = -1;
        double bestError1 = Double.MAX_VALUE;

        // Search for best fit using encoder 1 as primary reference
        int searchLimit1 = totalTeeth2;

        double teeth1Guess = rotatedTeeth1;

        for (double i = 0; i < searchLimit1; i += 1) {
            double teeth2ExpectedWithCurrentGuess = teeth1Guess % totalTeeth2;
            double error = Math.abs(rotatedTeeth2 - teeth2ExpectedWithCurrentGuess);

            // Handle wraparound: sometimes shorter path is going backwards
            double totalError = Math.min(error, totalTeeth2 - error);

            if (totalError < bestError1) {
                bestError1 = totalError;
                bestN1 = teeth1Guess;
            }
            teeth1Guess += totalTeeth1;
        }

        // Repeat search using encoder 2 as primary reference
        double bestN2 = -1;
        double bestError2 = Double.MAX_VALUE;
        int searchLimit2 = totalTeeth1;
        
        double teeth2Guess = rotatedTeeth2;

        for (double i = 0; i < searchLimit2; i += 1) {
            double teeth1ExpectedWithCurrentGuess = teeth2Guess % totalTeeth1;
            double error = Math.abs(rotatedTeeth1 - teeth1ExpectedWithCurrentGuess);

            // Handle wraparound
            double totalError = Math.min(error, totalTeeth1 - error);

            if (totalError < bestError2) {
                bestError2 = totalError;
                bestN2 = teeth2Guess;
            }
            teeth2Guess += totalTeeth2;
        }

        return Rotation2d.fromRotations((bestN1 + bestN2) / 2.0 / bigGearTeeth);
    }

    /**
     * SET TURRET TO DESIRED 0 POSITION, THEN CALL THIS TO LOG OFFSETS TO SMARTDASHBOARD
     * @param bigNTeeth
     * @param gear1teeth
     * @param gear2teeth
     * @param gear1rotations
     * @param gear2rotations
    */
    public static void smartDashboardLogABSOffsets(int gear1teeth, int gear2teeth, double gear1rotations, double gear2rotations) {
        // TODO: Make sure abs encoders wrap in other logic bc they may be big
        double middleTeeth = (gear1teeth * gear2teeth) / 2;

        double gear1remainder = middleTeeth % gear1teeth;
        double gear1offsetToApply = gear1remainder / gear1teeth - gear1rotations;
        SmartDashboard.putNumber("CRT/abs1OffsetRotation", gear1offsetToApply);

        double gear2remainder = middleTeeth % gear2teeth;
        double gear2offsetToApply = gear2remainder / gear2teeth - gear2rotations;
        SmartDashboard.putNumber("CRT/abs2OffsetRotation", gear2offsetToApply);
    }

    private static double wrapRotation(double rot) {
        while (rot > 1.0) rot -= 1.0;
        while (rot < 0.0) rot += 1.0;
        return rot;
    }
    /**
     * Runs automated tests to verify the Chinese Remainder Theorem implementation.
     */
    public static void runTests() {
        testOverallAngle(Rotation2d.fromDegrees(10.0), 100, 11, 13);
        testOverallAngle(Rotation2d.fromDegrees(45.0), 100, 13, 19);
        testOverallAngle(Rotation2d.fromDegrees(90.35), 100, 11, 13);
        testOverallAngle(Rotation2d.fromDegrees(180.0), 100, 11, 13);
        testOverallAngle(Rotation2d.fromDegrees(385.87), 100, 11, 17);

        // Test with encoder noise
        testOverallAngle(Rotation2d.fromDegrees(359.0), 100, 11, 13, 5.0);
        testOverallAngle(Rotation2d.fromDegrees(157.5), 100, 11, 13, 5.0);
    }

    /**
     * Tests the algorithm with a specific angle configuration and optional noise.
     * 
     * @param bigNAngle Known angle of the big gear to test
     * @param bigNTeeth Number of teeth on the big gear
     * @param gear1teeth Number of teeth on first encoder gear
     * @param gear2teeth Number of teeth on second encoder gear
     * @param noiseDegrees Optional noise to add to encoder readings (degrees)
     */
    // Main gear 100 teeth
    public static void testOverallAngle(Rotation2d bigNAngle, int bigNTeeth, int gear1teeth, int gear2teeth) {
        testOverallAngle(bigNAngle, bigNTeeth, gear1teeth, gear2teeth, 0.0);
    }
    
    public static void testOverallAngle(Rotation2d bigNAngle, int bigNTeeth, int gear1teeth, int gear2teeth, double noiseDegrees) {
        // Calculate expected encoder readings for the given big gear angle
        double gear1remainder = (bigNAngle.getRotations() * bigNTeeth) % gear1teeth;
        double gear2remainder = (bigNAngle.getRotations() * bigNTeeth) % gear2teeth;

        if (noiseDegrees > 0.0) {
            // could always do this, but saves some printing; cleaner output
            Random random = new Random();
            double error = random.nextGaussian(0.0, noiseDegrees) / 360.0 * gear1teeth;
            System.out.println("Gear 1: " + gear1remainder + " + " + error);
            gear1remainder += error;
            error = random.nextGaussian(0.0, noiseDegrees) / 360.0 * gear2teeth;
            System.out.println("Gear 2: " + gear2remainder + " + " + error);
            gear2remainder += error;
        }
        
        // Reconstruct angle from simulated encoder readings
        Rotation2d teeth = ChineseRemainder.findAngle(
                gear1remainder / gear1teeth, gear1teeth,
                gear2remainder / gear2teeth, gear2teeth, bigNTeeth);

        System.out.println("Testing: " + bigNAngle.getDegrees() + " degrees with (" + gear1teeth + ","
                + gear2teeth + "):   result = " + teeth.getDegrees() + " degrees");
    }

    public static void main(String[] args) {

        runTests();
    }
}