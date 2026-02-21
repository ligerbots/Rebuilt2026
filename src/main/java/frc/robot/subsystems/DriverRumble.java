// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverRumble extends SubsystemBase {   
    
    private final double RUMBLE_INTENSITY = 1; 
    
    private final XboxController m_xbox;
    private final BooleanSupplier m_turretFlip;

    Timer m_timer;

    public DriverRumble(XboxController xbox, BooleanSupplier turretFlip) {
        m_xbox = xbox;
        m_turretFlip = turretFlip;
        m_timer = new Timer();
    }
    
    @Override
    public void periodic() {
        if (!DriverStation.isTeleopEnabled()) {
            // not in Teleop and/or not Enabled
            // nothing to do - don't rumble
            m_xbox.setRumble(RumbleType.kBothRumble, 0);
            return;
        }
        
        boolean rumble = m_turretFlip.getAsBoolean();

        m_xbox.setRumble(RumbleType.kBothRumble, rumble ? RUMBLE_INTENSITY : 0);

        SmartDashboard.putBoolean("turret/rumble", rumble);
    }

    // public void rumble() {
    //     m_timer.reset();
    //     m_timer.start();
    // }
}
