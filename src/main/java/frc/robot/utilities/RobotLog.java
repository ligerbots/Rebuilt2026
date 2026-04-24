package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public final class RobotLog {
    private static final Map<String, DoubleLogEntry> DOUBLE_ENTRIES = new HashMap<>();
    private static final Map<String, BooleanLogEntry> BOOLEAN_ENTRIES = new HashMap<>();
    private static final Map<String, StringLogEntry> STRING_ENTRIES = new HashMap<>();
    private static final Map<String, Double> LAST_LOG_TIMESTAMPS_SEC = new HashMap<>();

    private RobotLog() {}

    public static void log(String key, double value) {
        DOUBLE_ENTRIES.computeIfAbsent(key, name -> new DoubleLogEntry(DataLogManager.getLog(), name)).append(value);
    }

    public static void log(String key, double value, double minPeriodSec) {
        if (shouldLog(key, minPeriodSec)) {
            log(key, value);
        }
    }

    public static void log(String key, boolean value) {
        BOOLEAN_ENTRIES.computeIfAbsent(key, name -> new BooleanLogEntry(DataLogManager.getLog(), name)).append(value);
    }

    public static void log(String key, boolean value, double minPeriodSec) {
        if (shouldLog(key, minPeriodSec)) {
            log(key, value);
        }
    }

    public static void log(String key, String value) {
        STRING_ENTRIES.computeIfAbsent(key, name -> new StringLogEntry(DataLogManager.getLog(), name)).append(value);
    }

    public static void log(String key, String value, double minPeriodSec) {
        if (shouldLog(key, minPeriodSec)) {
            log(key, value);
        }
    }

    private static boolean shouldLog(String key, double minPeriodSec) {
        double now = Timer.getFPGATimestamp();
        Double lastLogSec = LAST_LOG_TIMESTAMPS_SEC.get(key);
        if (lastLogSec != null && now - lastLogSec < minPeriodSec) {
            return false;
        }

        LAST_LOG_TIMESTAMPS_SEC.put(key, now);
        return true;
    }
}
