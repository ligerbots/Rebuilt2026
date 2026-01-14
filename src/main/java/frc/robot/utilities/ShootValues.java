package frc.robot.utilities;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ShootValues {
    public final double m_rpm;
    public final Rotation2d m_hoodAngle;

    /**
     * Creates a new ShootValues instance.
     * 
     * @param rpm       The shooter RPM
     * @param hoodAngle The hood angle as a Rotation2d object
     */
    public ShootValues(double rpm, Rotation2d hoodAngle) {
      this.m_rpm = rpm;
      this.m_hoodAngle = hoodAngle;
    }

    /**
     * Interpolates between two ShootValues using linear interpolation.
     * 
     * @param start The starting ShootValues
     * @param end   The ending ShootValues to interpolate towards
     * @param ratio The interpolation ratio (0.0 to 1.0)
     * @return A new ShootValues instance with interpolated values
     */
    public static ShootValues interpolate(ShootValues start, ShootValues end, double ratio) {
      double interpolatedRpm = start.m_rpm + (end.m_rpm - start.m_rpm) * ratio;
      // Interpolate hood angle using Rotation2d arithmetic: start + (end - start) * ratio
      Rotation2d interpolatedHoodAngle = start.m_hoodAngle.plus(end.m_hoodAngle.minus(start.m_hoodAngle).times(ratio));
      return new ShootValues(interpolatedRpm, interpolatedHoodAngle);
    }

    /**
     * Lookup table mapping distance (in meters) to shooter parameters.
     * Uses TreeMap for efficient range-based lookups and interpolation.
     */
    private static final TreeMap<Double, ShootValues> LOOKUP_TABLE = new TreeMap<>(Map.ofEntries(
        Map.entry(Units.inchesToMeters(52), new ShootValues(3000.0, Rotation2d.fromDegrees(55.0))),
        Map.entry(Units.inchesToMeters(104.3), new ShootValues(3000.0, Rotation2d.fromDegrees(37.0))),
        Map.entry(Units.inchesToMeters(137.4), new ShootValues(3100.0, Rotation2d.fromDegrees(30.5))),
        Map.entry(Units.inchesToMeters(146.5), new ShootValues(3400.0, Rotation2d.fromDegrees(29.5))),
        Map.entry(Units.inchesToMeters(168.0), new ShootValues(3850.0, Rotation2d.fromDegrees(26.0))),
        Map.entry(Units.inchesToMeters(224.0), new ShootValues(4600.0, Rotation2d.fromDegrees(21.75)))));

    /**
     * Retrieves shooter parameters for a given distance.
     * Performs linear interpolation if the distance falls between lookup table
     * entries.
     * 
     * @param distance The distance in meters
     * @return ShootValues containing RPM and hood angle, or null if distance is out
     *         of range
     */
    public static ShootValues getShootValues(double distance) {
      Map.Entry<Double, ShootValues> before = LOOKUP_TABLE.floorEntry(distance);
      Map.Entry<Double, ShootValues> after = LOOKUP_TABLE.ceilingEntry(distance);

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
      return ShootValues.interpolate(before.getValue(), after.getValue(), ratio);
    }
  }