package frc.robot;

public class Constants {
    public static final int FLYWHEEL_CAN_ID = 9;
    public static final int INTAKE_ROLLER_CAN_ID = 17;
    public static final int TURRET_CAN_ID = 10;
    public static final int CAN_CODER1 = 21;
    public static final int CAN_CODER2 = 22;
    // NOTE: TalonFX motors (all Krakens and Falcons) share the same ID space, so CANNOT repeat
    // CAN ID 1-8 for Talons are for the Swerve motors
    // CAN ID 1-4 for CANcoders are for the Swerve encoders

    // 9 - Shooter Flywheel
    // 10 - Shooter Turret
    // 11 - Shooter Hood
    // 12 - Shooter Kicker
    // 13, 14, 15 - Climber 
    // 16 Intake deploy
    // 18, 19 Transfer/Funnel
    // 20, Spare
}
