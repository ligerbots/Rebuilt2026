// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerSystem extends SubsystemBase {
    private final PowerDistribution m_powerDist;

    // private final static double K_T = 1.0 / 11.0;
    // private final static double K_I = 0.14;

    /** Creates a new PowerDistribution. */
    public PowerSystem() {
        m_powerDist = new PowerDistribution();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("logging/powerDist/totalCurrent", m_powerDist.getTotalCurrent());

        // If using below, make sure to update the CAN ID's
        // SmartDashboard.putNumber("powerDist/totalDriveCurrent", 
        //         m_powerDist.getCurrent(11) + m_powerDist.getCurrent(13) + 
        //         m_powerDist.getCurrent(15) + m_powerDist.getCurrent(16));
        // SmartDashboard.putNumber("powerDist/totalSteerCurrent", 
        //         m_powerDist.getCurrent(10) + m_powerDist.getCurrent(12) + 
        //         m_powerDist.getCurrent(14) + m_powerDist.getCurrent(17));
    }

    // private double breakerTemp(double oldTemp, double current, double deltaTime)
    // {
    //     double dT_dt = -K_T * oldTemp + K_I * current;
    //     return oldTemp + dT_dt * deltaTime;
    // }
}