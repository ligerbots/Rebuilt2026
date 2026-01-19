package frc.robot.utilities;

import java.util.Random;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Utility class that implements the Chinese Remainder Theorem to solve for absolute encoder positions
 * using two relative encoders with different gear ratios. This is commonly used in FRC robotics
 * to create high-resolution absolute position feedback by combining two encoders with coprime gear ratios.
 * 
 * The algorithm finds the unique solution within a given range that satisfies both encoder readings
 * simultaneously, effectively creating an absolute encoder system from relative encoders.
 */
public class ChineseRemainder {

    /**
     * Finds the absolute angle of a large gear using readings from two encoders with different gear ratios.
     * Uses the Chinese Remainder Theorem to solve for the unique position that satisfies both encoder readings.
     * 
     * This method works by searching for the number of teeth rotated that best matches both encoder readings,
     * accounting for the modular arithmetic nature of gear systems. It uses a brute-force search with
     * fine step resolution to find the optimal solution.
     * 
     * @param rotationsEnc1 The rotation reading from the first encoder (in rotations)
     * @param totalTeeth1 The total number of teeth that the first encoder sees per revolution of the big gear
     * @param rotationsEnc2 The rotation reading from the second encoder (in rotations)  
     * @param totalTeeth2 The total number of teeth that the second encoder sees per revolution of the big gear
     * @param bigGearTeeth The number of teeth on the large gear whose absolute position we want to determine
     * @return The absolute angle of the large gear as a Rotation2d
     */
    public static Rotation2d findAngle(Rotation2d rotationsEnc1, int totalTeeth1, Rotation2d rotationsEnc2, int totalTeeth2, int bigGearTeeth) {
        // Step 1: Convert encoder rotations to "teeth passed by" for easier math
        // If encoder 1 rotated 0.5 times and sees 20 teeth per big gear rotation, then 10 teeth have passed
        double rotatedTeeth1 = rotationsEnc1.getRotations() * totalTeeth1;
        double rotatedTeeth2 = rotationsEnc2.getRotations() * totalTeeth2;

        double bestN = -1;
        double bestError = Double.MAX_VALUE;

        // Step 2: Try different values of N (total teeth rotated on big gear) and find which fits both encoders best
        // We search from 0 to BigGearTeeth because after that, positions repeat (one full rotation)
        int searchLimit = totalTeeth1 * totalTeeth2;

        for (double I = 0; I < searchLimit; I += 1) {
            double teeth1Guess = I * totalTeeth1 + rotatedTeeth1;  // What encoder 1 should see

            double teeth2ExpectedWithCurrentGuess = teeth1Guess % totalTeeth2; // What encoder 2 should see based on encoder 1's guess
            // Step 4: Calculate how far off our prediction is from reality
            double error = Math.abs(rotatedTeeth2 - teeth2ExpectedWithCurrentGuess);

            // Step 5: Account for "wraparound" - sometimes the shorter path is going backwards
            // Example: if error is 18 out of 20 teeth, it's really just 2 teeth the other way
            double totalError = Math.min(error, totalTeeth2 - error);

            // Step 6: Keep track of the N value that gives us the smallest error
            if (totalError < bestError) {
                bestError = totalError;
                bestN = I;
            }
        }

        // Step 7: Convert best teeth count back to big gear rotations and return
        return Rotation2d.fromRotations((bestN*totalTeeth1 + rotatedTeeth1) / bigGearTeeth);
    }

    /**
     * Runs a series of automated tests to verify the correctness of the Chinese Remainder Theorem implementation.
     * Tests various angles using 11-tooth and 13-tooth gears with a 100-tooth main gear.
     * 
     * The test cases use coprime gear ratios (11 and 13) to ensure unambiguous solutions.
     * Each test simulates encoder readings for a known angle and verifies that the algorithm
     * can correctly reconstruct the original angle.
     */
    public static void runTests() {
        // AI generated tests for Chinese Remainder Theorem implementation. Good enough for sanity checking.
        // All tests use 11-tooth and 13-tooth gears (coprime for unambiguous solutions)
        testOverallAngle(Rotation2d.fromDegrees(10.0), 100, 11, 13);
        testOverallAngle(Rotation2d.fromDegrees(45.0), 100, 13, 19);
        testOverallAngle(Rotation2d.fromDegrees(90.35), 100, 11, 13);
        testOverallAngle(Rotation2d.fromDegrees(180.0), 100, 11, 13);
        // test with some other ratios
        testOverallAngle(Rotation2d.fromDegrees(385.87), 100, 11, 17);

        // test with a little noise on the encoders
        testOverallAngle(Rotation2d.fromDegrees(270.0), 100, 11, 13, 1.0);
        testOverallAngle(Rotation2d.fromDegrees(157.5), 100, 11, 13, 1.0);
    }

    /**
     * Tests the Chinese Remainder algorithm with a specific angle configuration.
     * Simulates encoder readings for a known big gear angle and verifies that the algorithm
     * can correctly reconstruct the original angle.
     * 
     * @param bigNAngle The known angle of the big gear to test with
     * @param bigNTeeth The number of teeth on the big gear
     * @param gear1teeth The number of teeth on the first encoder gear
     * @param gear2teeth The number of teeth on the second encoder gear
     */
    // Main gear 100 teeth
    public static void testOverallAngle(Rotation2d bigNAngle, int bigNTeeth, int gear1teeth, int gear2teeth) {
        testOverallAngle(bigNAngle, bigNTeeth, gear1teeth, gear2teeth, 0.0);
    }
    public static void testOverallAngle(Rotation2d bigNAngle, int bigNTeeth, int gear1teeth, int gear2teeth, double noiseDegrees) {
        // Calculate what the encoder readings should be for the given big gear angle
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
        // Use the algorithm to reconstruct the angle from the simulated encoder readings
        Rotation2d teeth = ChineseRemainder.findAngle(
                Rotation2d.fromRotations(gear1remainder / gear1teeth), gear1teeth,
                Rotation2d.fromRotations(gear2remainder / gear2teeth), gear2teeth, bigNTeeth);

        // Output the results for verification
        System.out.println("Testing: " + bigNAngle.getDegrees() + " degrees with (" + gear1teeth + ","
                + gear2teeth + "):   result = " + teeth.getDegrees() + " degrees");
    }
}