// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.HubShiftUtil;
import frc.robot.utilities.RobotLog;
import frc.robot.utilities.RateLimitedSmartDashboard;

public class DataLogger extends SubsystemBase {
    private static final double HUB_TIMING_DASHBOARD_PERIOD_SEC = 0.1;

    // private final PowerDistribution m_powerDist = new PowerDistribution();

    public DataLogger() {
    }

    @Override
    public void periodic() {
        // Power
        // RobotLog.log("power/totalCurrent", m_powerDist.getTotalCurrent());
        RobotLog.log("power/batteryVoltage", RobotController.getBatteryVoltage());

        // Network
        RobotLog.log("network/CAN Bus Utilization",
                RobotBase.isSimulation() ? 0.0 : RobotController.getCANStatus().percentBusUtilization);

        HubShiftUtil.ShiftInfo officialShiftInfo = HubShiftUtil.getOfficialShiftInfo();
        HubShiftUtil.ShiftInfo shiftedShiftInfo = HubShiftUtil.getShiftedShiftInfo(HubShiftUtil.getProjectileLeadTimeSec());
        boolean hubTimingRelevant = HubShiftUtil.isHubTimingRelevant();
        boolean hubActiveNow = officialShiftInfo.active();
        boolean clearToShoot = hubTimingRelevant && shiftedShiftInfo.active();

        // SmartDashboard
        RateLimitedSmartDashboard.putNumber("shoot/matchElapsedSec", HubShiftUtil.getMatchElapsedSec(), HUB_TIMING_DASHBOARD_PERIOD_SEC);
        RateLimitedSmartDashboard.putNumber("shoot/matchRemainingSec", HubShiftUtil.getMatchRemainingSec(), HUB_TIMING_DASHBOARD_PERIOD_SEC);
        RateLimitedSmartDashboard.putNumber("shoot/hubShiftElapsedSec", shiftedShiftInfo.elapsedTimeSec(), HUB_TIMING_DASHBOARD_PERIOD_SEC);
        RateLimitedSmartDashboard.putNumber("shoot/hubShiftRemainingSec", shiftedShiftInfo.remainingTimeSec(), HUB_TIMING_DASHBOARD_PERIOD_SEC);
        RateLimitedSmartDashboard.putBoolean("shoot/hubActiveNow", hubActiveNow, HUB_TIMING_DASHBOARD_PERIOD_SEC);
        RateLimitedSmartDashboard.putBoolean("shoot/clearToShoot", clearToShoot, HUB_TIMING_DASHBOARD_PERIOD_SEC);
        RateLimitedSmartDashboard.putBoolean("shoot/hubTimingRelevant", hubTimingRelevant, HUB_TIMING_DASHBOARD_PERIOD_SEC);

        // RobotLog
        RobotLog.log("shoot/hubShiftState", officialShiftInfo.currentShift().name());
        RobotLog.log("shoot/shiftedHubShiftState", shiftedShiftInfo.currentShift().name());
        RobotLog.log("shoot/hubActiveNow", hubActiveNow);
        RobotLog.log("shoot/clearToShoot", clearToShoot);
        RobotLog.log("shoot/hubTimingRelevant", hubTimingRelevant);
        RobotLog.log("shoot/matchElapsedSec", HubShiftUtil.getMatchElapsedSec());
        RobotLog.log("shoot/matchRemainingSec", HubShiftUtil.getMatchRemainingSec());
        RobotLog.log("shoot/hubShiftElapsedSec", shiftedShiftInfo.elapsedTimeSec());
        RobotLog.log("shoot/hubShiftRemainingSec", shiftedShiftInfo.remainingTimeSec());
        RobotLog.log("shoot/projectileLeadTimeSec", HubShiftUtil.getProjectileLeadTimeSec());
    }
}
