package frc.robot.utilities;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Manages ballistic lookup tables for shooter parameters.
 * Provides interpolation between data points for smooth shooter control.
 */
public class ShooterLookupTable { 
  private static final String LOOKUP_TABLE_DIRECTORY = "lookupTables/";
  private static final String LOOKUP_TABLE_EXTENSION = ".csv";

  private final TreeMap<Double, ShootValue> m_lookupTable;

  // Separator between values in each line (Distance/RPM/HoodAngle)
  private static final String SEPARATOR = ",";


  public ShooterLookupTable(String path) {
    m_lookupTable = loadLookupTableFromFile(path);
  }

  /**
   * Tuple-like structure containing shooter parameters.
   * Represents RPM and hood angle for a given distance.
   */
  public static class ShootValue {
    public final double flyRPM;
    public final double feedRPM;
    public final Rotation2d hoodAngle;

    /**
     * Creates a new ShootValue.
     * 
     * @param flyRPM       The shooter RPM
     * @param hoodAngle The hood angle as a Rotation2d object
     */
    public ShootValue(double flyRPM, double feedRPM, Rotation2d hoodAngle) {
      this.flyRPM = flyRPM;
      this.feedRPM = feedRPM;
      this.hoodAngle = hoodAngle;
    }
  }

  /**
   * Interpolates between two ShootValue instances using linear interpolation.
   * 
   * @param start The starting ShootValue
   * @param end   The ending ShootValue to interpolate towards
   * @param ratio The interpolation ratio (0.0 to 1.0)
   * @return A new ShootValue instance with interpolated values
   */
  private static ShootValue interpolate(ShootValue start, ShootValue end, double ratio) {
    double interpolatedFlyRpm = start.flyRPM + (end.flyRPM - start.flyRPM) * ratio;
    double feedRPM = start.feedRPM + (end.feedRPM - start.feedRPM) * ratio;

    // Interpolate hood angle using Rotation2d arithmetic: start + (end - start) * ratio
    Rotation2d interpolatedHoodAngle = start.hoodAngle.plus(end.hoodAngle.minus(start.hoodAngle).times(ratio));
    return new ShootValue(interpolatedFlyRpm, feedRPM, interpolatedHoodAngle);
  }

  /**
   * Retrieves shooter parameters for a given distance.
   * Performs linear interpolation if the distance falls between lookup table entries.
   * 
   * @param distance The distance in meters
   * @return ShootValue containing RPM and hood angle, or null if distance is out of range
   */
  public ShootValue getShootValues(double distance) {
    Map.Entry<Double, ShootValue> before = m_lookupTable.floorEntry(distance);
    Map.Entry<Double, ShootValue> after = m_lookupTable.ceilingEntry(distance);

    // Handle out-of-range distances (eg. below min or above max value in lookup table)
    if (before == null) {
      return after != null ? after.getValue() : null;
    }
    if (after == null) {
      return before.getValue();
    }

    double distanceDiff = after.getKey() - before.getKey();

    // Interpolate between two entries
    double ratio = (distance - before.getKey()) / distanceDiff;
    return ShooterLookupTable.interpolate(before.getValue(), after.getValue(), ratio);
  }


  /**
   * Loads a shooter lookup table from a file in the deploy/lookupTables/ directory.
   * 
   * File format: Distance(Inches)/RPM/HoodAngle(Degrees), newline-separated entries using "/" as delimiter.
   * 
   * @param fileName The name of the file (without extension) located in deploy/lookupTables/
   * @return A TreeMap representing the loaded lookup table, or null if an error occurs
   */
  private TreeMap<Double, ShootValue> loadLookupTableFromFile(String fileName) {
    try (BufferedReader br = new BufferedReader(
        new FileReader(
            new File(
                Filesystem.getDeployDirectory(), LOOKUP_TABLE_DIRECTORY + fileName + LOOKUP_TABLE_EXTENSION)))) {
      TreeMap<Double, ShootValue> parsedTable = new TreeMap<>();
      String line;
      
      while ((line = br.readLine()) != null) {
        // Trim and remove all non-numeric/separator characters
        String cleanedEntry = line.replaceAll("[^0-9." + SEPARATOR + "]", "").trim();
        
        // Skip empty entries
        if (cleanedEntry.isEmpty()) {
          continue;
        }
        
        // Parse the three separator-delimited values
        String[] values = cleanedEntry.split(SEPARATOR);
        if (values.length != 3) {
          continue; // Skip malformed entries
        }
        
        try {
          double distanceInches = Double.parseDouble(values[0]);
          double flyRPM = Double.parseDouble(values[1]);
          double feedRPM = Double.parseDouble(values[2]);
          double hoodAngleDegrees = Double.parseDouble(values[3]);
          
          // Convert distance from inches to meters
          double distanceMeters = Units.inchesToMeters(distanceInches);
          Rotation2d hoodAngle = Rotation2d.fromDegrees(hoodAngleDegrees);
          
          parsedTable.put(distanceMeters, new ShootValue(flyRPM, feedRPM, hoodAngle));
        } catch (NumberFormatException e) {
          // Skip entries with invalid number formats
          continue;
        }
      }
      
      return parsedTable.isEmpty() ? null : parsedTable;
    } catch (FileNotFoundException e) {
      // Auto-generated catch block
      System.err.println("Lookup table file not found: " + fileName + " " + e.getStackTrace().toString());
    } catch (IOException e) {
      // Auto-generated catch block
      System.err.println("Error reading lookup table file: " + fileName + " " + e.getStackTrace().toString());
    }
    return null;
  }
}