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
  private static final String LOOKUP_TABLE_EXTENSION = ".lookupTable";

  private final TreeMap<Double, ShootValue> m_lookupTable;

  public ShooterLookupTable(String path) {
    m_lookupTable = loadLookupTableFromFile(path);
  }

  /**
   * Tuple-like structure containing shooter parameters.
   * Represents RPM and hood angle for a given distance.
   */
  public static class ShootValue {
    public final double m_rpm;
    public final Rotation2d m_hoodAngle;

    /**
     * Creates a new ShootValue.
     * 
     * @param rpm       The shooter RPM
     * @param hoodAngle The hood angle as a Rotation2d object
     */
    public ShootValue(double rpm, Rotation2d hoodAngle) {
      this.m_rpm = rpm;
      this.m_hoodAngle = hoodAngle;
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
  public static ShootValue interpolate(ShootValue start, ShootValue end, double ratio) {
    double interpolatedRpm = start.m_rpm + (end.m_rpm - start.m_rpm) * ratio;
    // Interpolate hood angle using Rotation2d arithmetic: start + (end - start) * ratio
    Rotation2d interpolatedHoodAngle = start.m_hoodAngle.plus(end.m_hoodAngle.minus(start.m_hoodAngle).times(ratio));
    return new ShootValue(interpolatedRpm, interpolatedHoodAngle);
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
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();

      // Separator between values in each line (Distance/RPM/HoodAngle)
      String separator = "/";

      TreeMap<Double, ShootValue> parsedTable = new TreeMap<>();
      
      // Split by newlines and process each entry
      String[] entries = fileContent.split("\\n");
      for (String entry : entries) {
        // Trim and remove all non-numeric/separator characters
        String cleanedEntry = entry.replaceAll("[^0-9." + separator + "]", "").trim();
        
        // Skip empty entries
        if (cleanedEntry.isEmpty()) {
          continue;
        }
        
        // Parse the three separator-delimited values
        String[] values = cleanedEntry.split(separator);
        if (values.length != 3) {
          continue; // Skip malformed entries
        }
        
        try {
          double distanceInches = Double.parseDouble(values[0]);
          double rpm = Double.parseDouble(values[1]);
          double hoodAngleDegrees = Double.parseDouble(values[2]);
          
          // Convert distance from inches to meters
          double distanceMeters = Units.inchesToMeters(distanceInches);
          Rotation2d hoodAngle = Rotation2d.fromDegrees(hoodAngleDegrees);
          
          parsedTable.put(distanceMeters, new ShootValue(rpm, hoodAngle));
        } catch (NumberFormatException e) {
          // Skip entries with invalid number formats
          continue;
        }
      }
      
      return parsedTable.isEmpty() ? null : parsedTable;
    } catch (FileNotFoundException e) {
      // Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // Auto-generated catch block
      e.printStackTrace();
    }
    return null;
  }
}