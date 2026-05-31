// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.HubShiftUtil;
import frc.robot.utilities.RobotLog;

public class DataLogger extends SubsystemBase {
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
        SmartDashboard.putNumber("shoot/matchElapsedSec", HubShiftUtil.getMatchElapsedSec());
        SmartDashboard.putNumber("shoot/matchRemainingSec", HubShiftUtil.getMatchRemainingSec());
        SmartDashboard.putNumber("shoot/hubShiftElapsedSec", shiftedShiftInfo.elapsedTimeSec());
        SmartDashboard.putNumber("shoot/hubShiftRemainingSec", shiftedShiftInfo.remainingTimeSec());
        SmartDashboard.putBoolean("shoot/hubActiveNow", hubActiveNow);
        SmartDashboard.putBoolean("shoot/clearToShoot", clearToShoot);
        SmartDashboard.putBoolean("shoot/hubTimingRelevant", hubTimingRelevant);

        // RobotLog
        RobotLog.log("shoot/hubShiftState", officialShiftInfo.currentShift().name());
        RobotLog.log("shoot/shiftedHubShiftState", shiftedShiftInfo.currentShift().name());
        RobotLog.log("shoot/projectileLeadTimeSec", HubShiftUtil.getProjectileLeadTimeSec());
    }
}
