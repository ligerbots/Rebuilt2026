// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.utilities.ShooterLookupTable;
import frc.robot.utilities.ShooterLookupTable.ShootValue;

public class Shooter extends SubsystemBase {
    public enum ShotType {
        HUB,
        PASS,
        TEST,   // test shot using SmartDashboard values. See Shoot cmd
        AUTO    // auto select hub vs pass
    }
    
    private static final String HUB_LOOKUP_TABLE_FILE = "hub_shooting_lookup_table"; 
    private static final String SHUTTLE_LOOKUP_TABLE_FILE = "shuttle_shooting_lookup_table";
    
    private final ShooterLookupTable m_hubShooterLookupTable;
    private final ShooterLookupTable m_shuttleShooterLookupTable;
    
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
    
    // @Override
    // public void periodic() {
    // }

    public Flywheel getFlywheel() {
        return m_flywheel;
    }

    public Hood getHood() {
        return m_hood;
    }
    
    /**
    * Stops all shooter-related subsystems.
    */
    public void stop() {
        m_flywheel.stop();
        m_hood.setAngle(Rotation2d.kZero);
    }
    
    public boolean onTarget() {
        return m_flywheel.onTarget() && m_hood.onTarget();
    }

    public void setShootValues(ShootValue shootValues) {
        // Phase 2: Set shooter speed, and hood angle
        m_flywheel.setRPM(shootValues.flyRPM);
        m_hood.setAngle(shootValues.hoodAngle);
    }

    /**
    * Sets the target distance for the shooter to use when calculating ballistic parameters.
    * 
    * @param distanceMeters The target distance in meters
    */
    public ShootValue getShootValue(double distanceMeters, ShotType shotType) {
        // Calculate distance to target and retrieve shooter hood angle and speed from shot type.
        if (shotType == ShotType.PASS)
            return m_shuttleShooterLookupTable.getShootValues(distanceMeters);
        return m_hubShooterLookupTable.getShootValues(distanceMeters);
    }
}
