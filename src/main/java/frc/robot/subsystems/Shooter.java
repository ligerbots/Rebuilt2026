// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ShooterLookupTable;

public class Shooter extends SubsystemBase {
    
    public enum ShooterState {
        IDLE,
        SPINNING_UP,
        READY_TO_SHOOT
    }
    
    public enum ShotType {
        HUB_SHOT,
        PASSING
    }
    
    private ShooterState m_currentState = ShooterState.IDLE;
    private ShotType m_shootType = ShotType.HUB_SHOT;
    
    private static final String HUB_LOOKUP_TABLE_FILE = "hub_shooting_lookup_table.csv";
    private static final String SHUTTLE_LOOKUP_TABLE_FILE = "shuttle_shooting_lookup_table.csv";
    
    private final ShooterLookupTable m_hubShooterLookupTable;
    private final ShooterLookupTable m_shuttleShooterLookupTable;
    
    private double m_targetDistanceMeters = 0.0;
    ShooterLookupTable.ShootValue m_shootValues;
    
    private Hood m_hood;
    private Flywheel m_flywheel;
    
    /**
    * Constructs a Shooter subsystem with lookup tables for hub and shuttle shooting.
    *
    * @param hubLookupTableFileName the file name for the lookup table for hub shooting calculations
    * @param shuttleLookupTableFileName the file name for the lookup table for shuttle shooting calculations
    */
    public Shooter() {
        m_hubShooterLookupTable = new ShooterLookupTable(HUB_LOOKUP_TABLE_FILE);
        m_shuttleShooterLookupTable = new ShooterLookupTable(SHUTTLE_LOOKUP_TABLE_FILE);
        
        m_hood = new Hood();
        m_flywheel = new Flywheel();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        // If in IDLE state, do nothing (skip shooting logic below)
        if (m_currentState == ShooterState.IDLE) {
            return;
        }
        
        // Handle case where distance is out of range
        if (m_shootValues == null) {
            return;
        }
        
        // Phase 2: Set shooter speed, and hood angle
        m_flywheel.setRPM(m_shootValues.rpm);
        m_hood.setAngle(m_shootValues.hoodAngle);
        
        // Phase 3: Check if all subsystems are at target values before running feeder
        if (m_flywheel.onTarget() && m_hood.onTarget()) {
            m_currentState = ShooterState.READY_TO_SHOOT;
        } else {
            m_currentState = ShooterState.SPINNING_UP;
        }
    }
    
    /**
    * Stops all shooter-related subsystems.
    */
    public void stop() {
        m_currentState = ShooterState.IDLE;
        m_flywheel.stop();
        m_hood.setAngle(Rotation2d.fromDegrees(0));
    }
    
    // Pass through routines, needed for testing and tuning
    public void setFlywheelVoltage(double voltage) {
        m_flywheel.setVoltage(voltage);
    }
    
    public void setHoodAngle(Rotation2d angle) {
        m_hood.setAngle(angle);
    }
    
    /**
    * Sets the target distance for the shooter to use when calculating ballistic parameters.
    * 
    * @param distanceMeters The target distance in meters
    */
    public void setDistanceToTarget(double distanceMeters) {
        m_targetDistanceMeters = distanceMeters;

        // Calculate distance to target and retrieve shooter hood angle and speed from shot type.
        if (m_shootType == ShotType.HUB_SHOT) {
            m_shootValues = m_hubShooterLookupTable.getShootValues(m_targetDistanceMeters);
        } else {
            m_shootValues = m_shuttleShooterLookupTable.getShootValues(m_targetDistanceMeters);
        }
    }
    
    /**
    * Gets the current state of the shooter.
    * 
    * @return The current ShooterState (IDLE, SPINNING_UP, or READY_TO_SHOOT)
    */
    public ShooterState getCurrentState() {
        return m_currentState;
    }
    
    public void setShootType(ShotType shootType) {
        m_shootType = shootType;
    }
}
