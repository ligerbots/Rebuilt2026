// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DataLogger extends SubsystemBase {
    // private final PowerDistribution m_powerDist = new PowerDistribution();

    public DataLogger() {
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("power/totalCurrent", m_powerDist.getTotalCurrent());  
        SmartDashboard.putNumber("power/batteryVoltage", RobotController.getBatteryVoltage());  

        SmartDashboard.putNumber("network/CAN Bus Utilization", RobotController.getCANStatus().percentBusUtilization);
    }
}