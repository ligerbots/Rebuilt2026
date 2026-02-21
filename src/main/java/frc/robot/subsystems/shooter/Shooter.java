// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ShooterLookupTable;
import frc.robot.utilities.ShooterLookupTable.ShootValue;

public class Shooter extends SubsystemBase {
    public enum ShotType {
        HUB,
        PASS,
        TEST,   // test shot using SmartDashboard values. See Shoot cmd
        AUTO    // auto select hub vs pass
    }
    
    private static final String HUB_LOOKUP_TABLE_FILE = "hub_shot_lookup_table.csv"; 
    private static final String PASS_LOOKUP_TABLE_FILE = "pass_shot_lookup_table.csv";
    
    private final ShooterLookupTable m_hubShotLookupTable;
    private final ShooterLookupTable m_passShotLookupTable;
    private double m_flyFudge = 1.0;
    private static final double FUDGE_INCREMENT = 0.05;
    private double m_hoodFudgeDegree = 0.0;
    private static final double HOOD_FUDGE_INCREMENT_DEGREES = 0.5;
    
    private Hood m_hood;
    private Flywheel m_flywheel;

    /**
    * Constructs a Shooter subsystem with lookup tables for hub and shuttle shooting.
    *
    * @param hubLookupTableFileName the file name for the lookup table for hub shooting calculations
    * @param shuttleLookupTableFileName the file name for the lookup table for shuttle shooting calculations
    */
    public Shooter() {
        m_hubShotLookupTable = new ShooterLookupTable(HUB_LOOKUP_TABLE_FILE);
        m_passShotLookupTable = new ShooterLookupTable(PASS_LOOKUP_TABLE_FILE);
        
        m_hood = new Hood();
        m_flywheel = new Flywheel();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/flywheelFudge", m_flyFudge);
        SmartDashboard.putNumber("shooter/hoodFudge", m_hoodFudgeDegree);
    }

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

    public void increaseFlyFudge() {
        m_flyFudge += FUDGE_INCREMENT;
    }

     public void decreaseFlyFudge() {
        m_flyFudge -= FUDGE_INCREMENT;
    }

    public void increaseHoodFudge() {
        m_hoodFudgeDegree += HOOD_FUDGE_INCREMENT_DEGREES;
    }

     public void decreaseHoodFudge() {
        m_hoodFudgeDegree -= HOOD_FUDGE_INCREMENT_DEGREES;
    }


    /**
    * Sets the target distance for the shooter to use when calculating ballistic parameters.
    * 
    * @param distanceMeters The target distance in meters
    */
    public ShootValue getShootValue(double distanceMeters, ShotType shotType) {
        // Calculate distance to target and retrieve shooter hood angle and speed from shot type.
        if (shotType == ShotType.PASS)
            return m_passShotLookupTable.getShootValues(distanceMeters);

        ShootValue shootValue = m_hubShotLookupTable.getShootValues(distanceMeters);
        shootValue.flyRPM *= m_flyFudge;
        shootValue.hoodAngle = shootValue.hoodAngle.plus(Rotation2d.fromDegrees(m_hoodFudgeDegree));

        return shootValue;
    }
}
