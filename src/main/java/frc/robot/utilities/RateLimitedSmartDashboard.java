package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class RateLimitedSmartDashboard {
    private static final Map<String, Double> LAST_PUBLISH_TIMESTAMPS_SEC = new HashMap<>();

    private RateLimitedSmartDashboard() {}

    public static void putNumber(String key, double value, double minPeriodSec) {
        if (shouldPublish(key, minPeriodSec)) {
            SmartDashboard.putNumber(key, value);
        }
    }

    public static void putBoolean(String key, boolean value, double minPeriodSec) {
        if (shouldPublish(key, minPeriodSec)) {
            SmartDashboard.putBoolean(key, value);
        }
    }

    private static boolean shouldPublish(String key, double minPeriodSec) {
        double now = Timer.getFPGATimestamp();
        Double lastPublishSec = LAST_PUBLISH_TIMESTAMPS_SEC.get(key);
        if (lastPublishSec != null && now - lastPublishSec < minPeriodSec) {
            return false;
        }

        LAST_PUBLISH_TIMESTAMPS_SEC.put(key, now);
        return true;
    }
}
