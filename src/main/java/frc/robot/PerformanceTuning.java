package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class PerformanceTuning {
    private static final Map<String, Integer> LOOP_COUNTERS = new HashMap<>();

    private PerformanceTuning() {}

    public static void publishDefaults() {
        SmartDashboard.setDefaultBoolean("perf/visionEnabled", true);
        SmartDashboard.setDefaultBoolean("perf/visionThrottleEnabled", true);
        SmartDashboard.setDefaultNumber("perf/visionEveryLoops", 3);
        SmartDashboard.setDefaultBoolean("perf/visionFrameCapEnabled", true);
        SmartDashboard.setDefaultNumber("perf/visionMaxFramesPerCamera", 2);
        SmartDashboard.setDefaultBoolean("perf/visionFieldPlotsEnabled", false);

        SmartDashboard.setDefaultBoolean("perf/telemetryEnabled", true);
        SmartDashboard.setDefaultBoolean("perf/telemetryThrottleEnabled", true);
        SmartDashboard.setDefaultNumber("perf/telemetryEveryLoops", 3);
        SmartDashboard.setDefaultBoolean("perf/telemetrySignalLoggerEnabled", true);
        SmartDashboard.setDefaultBoolean("perf/telemetryFieldEnabled", false);
        SmartDashboard.setDefaultBoolean("perf/telemetryMechanismsEnabled", false);

        SmartDashboard.setDefaultBoolean("perf/drivetrainDashboardEnabled", true);
        SmartDashboard.setDefaultBoolean("perf/drivetrainDashboardThrottleEnabled", true);
        SmartDashboard.setDefaultNumber("perf/drivetrainDashboardEveryLoops", 3);

        SmartDashboard.setDefaultBoolean("perf/dataLoggerEnabled", true);
        SmartDashboard.setDefaultBoolean("perf/dataLoggerThrottleEnabled", true);
        SmartDashboard.setDefaultNumber("perf/dataLoggerEveryLoops", 5);

        SmartDashboard.setDefaultBoolean("perf/flywheelDashboardEnabled", true);
        SmartDashboard.setDefaultBoolean("perf/flywheelDashboardThrottleEnabled", true);
        SmartDashboard.setDefaultNumber("perf/flywheelDashboardEveryLoops", 5);

        SmartDashboard.setDefaultBoolean("perf/turretDashboardEnabled", true);
        SmartDashboard.setDefaultBoolean("perf/turretDashboardThrottleEnabled", true);
        SmartDashboard.setDefaultNumber("perf/turretDashboardEveryLoops", 5);

        SmartDashboard.setDefaultBoolean("perf/feederDashboardEnabled", true);
        SmartDashboard.setDefaultBoolean("perf/feederDashboardThrottleEnabled", true);
        SmartDashboard.setDefaultNumber("perf/feederDashboardEveryLoops", 5);

        SmartDashboard.setDefaultBoolean("perf/intakeRollerDashboardEnabled", true);
        SmartDashboard.setDefaultBoolean("perf/intakeRollerDashboardThrottleEnabled", true);
        SmartDashboard.setDefaultNumber("perf/intakeRollerDashboardEveryLoops", 5);

        SmartDashboard.setDefaultBoolean("perf/shootDashboardEnabled", true);
        SmartDashboard.setDefaultBoolean("perf/shootReducedLookaheadEnabled", true);
        SmartDashboard.setDefaultNumber("perf/shootLookaheadMaxIterations", 8);
        SmartDashboard.setDefaultBoolean("perf/shootPlotEnabled", RobotBase.isSimulation());
    }

    public static boolean shouldRunVisionThisLoop() {
        return gatedSection(
            "vision",
            "perf/visionEnabled", true,
            "perf/visionThrottleEnabled", true,
            "perf/visionEveryLoops", 3
        );
    }

    public static int getVisionMaxFramesPerCamera() {
        if (!SmartDashboard.getBoolean("perf/visionFrameCapEnabled", true)) {
            return -1;
        }
        return positiveInt("perf/visionMaxFramesPerCamera", 2);
    }

    public static boolean isVisionFieldPlotsEnabled() {
        return SmartDashboard.getBoolean("perf/visionFieldPlotsEnabled", false);
    }

    public static boolean shouldPublishTelemetryThisLoop() {
        return gatedSection(
            "telemetry",
            "perf/telemetryEnabled", true,
            "perf/telemetryThrottleEnabled", true,
            "perf/telemetryEveryLoops", 3
        );
    }

    public static boolean isTelemetrySignalLoggerEnabled() {
        return SmartDashboard.getBoolean("perf/telemetrySignalLoggerEnabled", true);
    }

    public static boolean isTelemetryFieldEnabled() {
        return SmartDashboard.getBoolean("perf/telemetryFieldEnabled", false);
    }

    public static boolean isTelemetryMechanismsEnabled() {
        return SmartDashboard.getBoolean("perf/telemetryMechanismsEnabled", false);
    }

    public static boolean shouldPublishDrivetrainDashboardThisLoop() {
        return gatedSection(
            "drivetrainDashboard",
            "perf/drivetrainDashboardEnabled", true,
            "perf/drivetrainDashboardThrottleEnabled", true,
            "perf/drivetrainDashboardEveryLoops", 3
        );
    }

    public static boolean shouldPublishDataLoggerThisLoop() {
        return gatedSection(
            "dataLogger",
            "perf/dataLoggerEnabled", true,
            "perf/dataLoggerThrottleEnabled", true,
            "perf/dataLoggerEveryLoops", 5
        );
    }

    public static boolean shouldPublishFlywheelDashboardThisLoop() {
        return gatedSection(
            "flywheelDashboard",
            "perf/flywheelDashboardEnabled", true,
            "perf/flywheelDashboardThrottleEnabled", true,
            "perf/flywheelDashboardEveryLoops", 5
        );
    }

    public static boolean shouldPublishTurretDashboardThisLoop() {
        return gatedSection(
            "turretDashboard",
            "perf/turretDashboardEnabled", true,
            "perf/turretDashboardThrottleEnabled", true,
            "perf/turretDashboardEveryLoops", 5
        );
    }

    public static boolean shouldPublishFeederDashboardThisLoop() {
        return gatedSection(
            "feederDashboard",
            "perf/feederDashboardEnabled", true,
            "perf/feederDashboardThrottleEnabled", true,
            "perf/feederDashboardEveryLoops", 5
        );
    }

    public static boolean shouldPublishIntakeRollerDashboardThisLoop() {
        return gatedSection(
            "intakeRollerDashboard",
            "perf/intakeRollerDashboardEnabled", true,
            "perf/intakeRollerDashboardThrottleEnabled", true,
            "perf/intakeRollerDashboardEveryLoops", 5
        );
    }

    public static boolean isShootDashboardEnabled() {
        return SmartDashboard.getBoolean("perf/shootDashboardEnabled", true);
    }

    public static int getShootLookaheadMaxIterations() {
        if (!SmartDashboard.getBoolean("perf/shootReducedLookaheadEnabled", true)) {
            return 20;
        }
        return positiveInt("perf/shootLookaheadMaxIterations", 8);
    }

    public static boolean isShootPlotEnabled() {
        return SmartDashboard.getBoolean("perf/shootPlotEnabled", RobotBase.isSimulation());
    }

    private static boolean gatedSection(
        String counterKey,
        String enabledKey,
        boolean enabledDefault,
        String throttleKey,
        boolean throttleDefault,
        String loopsKey,
        int loopsDefault
    ) {
        if (!SmartDashboard.getBoolean(enabledKey, enabledDefault)) {
            return false;
        }

        if (!SmartDashboard.getBoolean(throttleKey, throttleDefault)) {
            return true;
        }

        return shouldRunEvery(counterKey, positiveInt(loopsKey, loopsDefault));
    }

    private static boolean shouldRunEvery(String counterKey, int everyLoops) {
        if (everyLoops <= 1) {
            return true;
        }

        int counter = LOOP_COUNTERS.getOrDefault(counterKey, 0);
        boolean shouldRun = counter == 0;
        LOOP_COUNTERS.put(counterKey, (counter + 1) % everyLoops);
        return shouldRun;
    }

    private static int positiveInt(String key, int fallback) {
        return Math.max(1, (int) Math.round(SmartDashboard.getNumber(key, fallback)));
    }
}
