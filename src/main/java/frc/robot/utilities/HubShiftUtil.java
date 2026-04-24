package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public final class HubShiftUtil {
    private static final double NO_PROJECTILE_LEAD_TIME_SEC = 0.0;

    public enum ShiftState {
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        AUTO,
        DISABLED
    }

    public record ShiftInfo(ShiftState currentShift, double elapsedTimeSec, double remainingTimeSec, boolean active) {}

    private static final Timer MATCH_TIMER = new Timer();
    private static final ShiftState[] SHIFT_STATES = ShiftState.values();

    private static final double AUTO_DURATION_SEC = 20.0;
    private static final double TELEOP_DURATION_SEC = 140.0;
    private static final double HUB_SHIFT_LEAD_BUFFER_SEC = 0.0;

    private static final double[] SHIFT_START_TIMES_SEC = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] SHIFT_END_TIMES_SEC = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    private static final boolean[] ACTIVE_FIRST_SCHEDULE = {true, true, false, true, false, true};
    private static final boolean[] INACTIVE_FIRST_SCHEDULE = {true, false, true, false, true, true};

    private static double s_projectileLeadTimeSec = NO_PROJECTILE_LEAD_TIME_SEC;
    private static boolean s_hubTimingRelevant = false;

    private HubShiftUtil() {}

    public static void initialize() {
        MATCH_TIMER.restart();
    }

    public static void disable() {
        MATCH_TIMER.stop();
        MATCH_TIMER.reset();
        clearShotContext();
    }

    public static void setShotContext(double projectileLeadTimeSec, boolean hubTimingRelevant) {
        s_projectileLeadTimeSec = Math.max(0.0, projectileLeadTimeSec);
        s_hubTimingRelevant = hubTimingRelevant;
    }

    public static void clearShotContext() {
        s_projectileLeadTimeSec = NO_PROJECTILE_LEAD_TIME_SEC;
        s_hubTimingRelevant = false;
    }

    public static double getProjectileLeadTimeSec() {
        return s_projectileLeadTimeSec;
    }

    public static boolean isHubTimingRelevant() {
        return s_hubTimingRelevant;
    }

    public static double getMatchElapsedSec() {
        return MATCH_TIMER.get();
    }

    public static double getMatchRemainingSec() {
        if (DriverStation.isAutonomousEnabled()) {
            return Math.max(0.0, AUTO_DURATION_SEC - MATCH_TIMER.get());
        }
        if (DriverStation.isTeleopEnabled()) {
            return Math.max(0.0, TELEOP_DURATION_SEC - MATCH_TIMER.get());
        }
        return 0.0;
    }

    public static Alliance getFirstActiveAlliance() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        String message = DriverStation.getGameSpecificMessage();
        if (!message.isEmpty()) {
            char character = message.charAt(0);
            if (character == 'R') {
                return Alliance.Blue;
            } else if (character == 'B') {
                return Alliance.Red;
            }
        }

        return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    public static ShiftInfo getOfficialShiftInfo() {
        return getOfficialShiftInfo(false);
    }

    public static ShiftInfo getOfficialShiftInfo(boolean override) {
        return getShiftInfo(getSchedule(override), SHIFT_START_TIMES_SEC, SHIFT_END_TIMES_SEC, override);
    }

    public static ShiftInfo getShiftedShiftInfo(double projectileLeadTimeSec) {
        return getShiftedShiftInfo(projectileLeadTimeSec, false);
    }

    public static ShiftInfo getShiftedShiftInfo(double projectileLeadTimeSec, boolean override) {
        boolean[] shiftSchedule = getSchedule(override);
        double approachingActiveFudge = -Math.max(0.0, projectileLeadTimeSec + HUB_SHIFT_LEAD_BUFFER_SEC);

        if (shiftSchedule[1]) {
            double[] shiftedStartTimesSec = {0.0, 10.0, 35.0, 60.0 + approachingActiveFudge, 85.0, 110.0 + approachingActiveFudge};
            double[] shiftedEndTimesSec = {10.0, 35.0, 60.0 + approachingActiveFudge, 85.0, 110.0 + approachingActiveFudge, 140.0};
            return getShiftInfo(shiftSchedule, shiftedStartTimesSec, shiftedEndTimesSec, override);
        }

        double[] shiftedStartTimesSec = {0.0, 10.0, 35.0 + approachingActiveFudge, 60.0, 85.0 + approachingActiveFudge, 110.0};
        double[] shiftedEndTimesSec = {10.0, 35.0 + approachingActiveFudge, 60.0, 85.0 + approachingActiveFudge, 110.0, 140.0};
        return getShiftInfo(shiftSchedule, shiftedStartTimesSec, shiftedEndTimesSec, override);
    }

    private static boolean[] getSchedule(boolean override) {
        Alliance startAlliance = getFirstActiveAlliance();
        boolean activeFirst = startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue);
        if (override) {
            activeFirst = !activeFirst;
        }
        return activeFirst ? ACTIVE_FIRST_SCHEDULE : INACTIVE_FIRST_SCHEDULE;
    }

    private static ShiftInfo getShiftInfo(boolean[] currentSchedule, double[] shiftStartTimesSec,
            double[] shiftEndTimesSec, boolean override) {
        double currentTimeSec = MATCH_TIMER.get();
        double stateTimeElapsedSec = currentTimeSec;
        double stateTimeRemainingSec = 0.0;
        boolean active = false;
        ShiftState currentShift = ShiftState.DISABLED;

        if (DriverStation.isAutonomousEnabled()) {
            stateTimeRemainingSec = Math.max(0.0, AUTO_DURATION_SEC - currentTimeSec);
            active = true;
            currentShift = ShiftState.AUTO;
        } else if (DriverStation.isTeleopEnabled()) {
            int currentShiftIndex = shiftStartTimesSec.length - 1;
            for (int i = 0; i < shiftStartTimesSec.length; i++) {
                if (currentTimeSec >= shiftStartTimesSec[i] && currentTimeSec < shiftEndTimesSec[i]) {
                    currentShiftIndex = i;
                    break;
                }
            }

            stateTimeElapsedSec = currentTimeSec - shiftStartTimesSec[currentShiftIndex];
            stateTimeRemainingSec = shiftEndTimesSec[currentShiftIndex] - currentTimeSec;

            if (currentShiftIndex > 0 && currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
                stateTimeElapsedSec = currentTimeSec - shiftStartTimesSec[currentShiftIndex - 1];
            }

            if (currentShiftIndex < shiftEndTimesSec.length - 1
                    && currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
                stateTimeRemainingSec = shiftEndTimesSec[currentShiftIndex + 1] - currentTimeSec;
            }

            active = currentSchedule[currentShiftIndex];
            currentShift = SHIFT_STATES[currentShiftIndex];
        }

        return new ShiftInfo(
                currentShift,
                Math.max(0.0, stateTimeElapsedSec),
                Math.max(0.0, stateTimeRemainingSec),
                active || override);
    }
}
