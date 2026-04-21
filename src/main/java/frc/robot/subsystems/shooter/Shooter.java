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
        OPPOSITE_ZONE,
        TEST,    // test shot using SmartDashboard values. See Shoot cmd
        AUTO,    // auto select hub vs pass
        FIXED    // fixed distance and turret heading
    }
    
    private static final String HUB_LOOKUP_TABLE_FILE = "hub_shot_lookup_table.csv"; 
    private static final String PASS_LOOKUP_TABLE_FILE = "pass_shot_lookup_table.csv";
    private static final String OPPOSITE_ZONE_LOOKUP_TABLE_FILE = "opposite_zone_shot_lookup_table.csv";

    private final ShooterLookupTable m_hubShotLookupTable;
    private final ShooterLookupTable m_passShotLookupTable;
    private final ShooterLookupTable m_oppositeZoneShotLookupTable;

    private boolean m_passNeutral = true;

    // Manual adjust on the flywheel RPM
    // NOTE: this is a multiplicative change: +5%, +10%, etc
    private double m_flyFudge = 1.0;
    private static final double FLY_FUDGE_INCREMENT = 0.02;

    // Manual adjust on the flywheel RPM
    // NOTE: this is a multiplicative change: +5%, +10%, etc
    private double m_feedFudge = 1.0;
    private static final double FEED_FUDGE_INCREMENT = 0.05;

    // Manual adjust on the hood angle (additive)
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
        m_oppositeZoneShotLookupTable = new ShooterLookupTable(OPPOSITE_ZONE_LOOKUP_TABLE_FILE);

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
        m_flyFudge += FLY_FUDGE_INCREMENT;
    }

    public void decreaseFlyFudge() {
        m_flyFudge -= FLY_FUDGE_INCREMENT;
    }

    public void increaseFeedFudge() {
        m_feedFudge += FEED_FUDGE_INCREMENT;
    }

    public void decreaseFeedFudge() {
        m_feedFudge -= FEED_FUDGE_INCREMENT;
    }

    public void increaseHoodFudge() {
        m_hoodFudgeDegree += HOOD_FUDGE_INCREMENT_DEGREES;
    }

    public void decreaseHoodFudge() {
        m_hoodFudgeDegree -= HOOD_FUDGE_INCREMENT_DEGREES;
    }

    public void setPassNeutral(boolean passNeutral) {
        m_passNeutral = passNeutral;
    }

    public boolean getPassNeutral() {
        return m_passNeutral;
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
        if (shotType == ShotType.OPPOSITE_ZONE)
            return m_oppositeZoneShotLookupTable.getShootValues(distanceMeters);

        ShootValue shootValue = m_hubShotLookupTable.getShootValues(distanceMeters);
        shootValue.flyRPM *= m_flyFudge;
        shootValue.feedRPM *= m_feedFudge;
        shootValue.hoodAngle = shootValue.hoodAngle.plus(Rotation2d.fromDegrees(m_hoodFudgeDegree));

        return shootValue;
    }
}
