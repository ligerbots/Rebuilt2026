// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PerformanceTuning;

public class DataLogger extends SubsystemBase {
    // private final PowerDistribution m_powerDist = new PowerDistribution();
    private final DoubleLogEntry m_batteryVoltageLog =
            new DoubleLogEntry(DataLogManager.getLog(), "LOG:/SmartDashboard/power/batteryVoltage");
    private final DoubleLogEntry m_canUtilizationLog =
            new DoubleLogEntry(DataLogManager.getLog(), "LOG:/SmartDashboard/network/CAN Bus Utilization");

    public DataLogger() {
    }

    @Override
    public void periodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        double canUtilization = RobotBase.isSimulation() ? 0.0 : RobotController.getCANStatus().percentBusUtilization;

        if (PerformanceTuning.isDataLoggerDirectLogEnabled()) {
            m_batteryVoltageLog.append(batteryVoltage);
            m_canUtilizationLog.append(canUtilization);
        }

        if (!PerformanceTuning.shouldPublishDataLoggerThisLoop()) {
            return;
        }

        // SmartDashboard.putNumber("power/totalCurrent", m_powerDist.getTotalCurrent());  
        SmartDashboard.putNumber("power/batteryVoltage", batteryVoltage);
        SmartDashboard.putNumber("network/CAN Bus Utilization", canUtilization);
    }
}
