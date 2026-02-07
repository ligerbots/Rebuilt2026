package frc.robot;

public class Constants {
    public static final int FLYWHEEL_CAN_ID = 9;
    public static final int FLYWHEEL_FOLLOWER_CAN_ID = 10;
    public static final int HOOD_CAN_ID = 11;
    public static final int LINEAR_EXTENSION_CAN_ID = 12;
    public static final int CLIMBER_LEFT_MOTOR_CAN_ID = 13;
    public static final int CLIMBER_RIGHT_MOTOR_CAN_ID = 14;
    public static final int CHAIN_CLIMBER_MOTOR_CAN_ID = 15; 
    public static final int INTAKE_DEPLOY_ID = 16;
    public static final int INTAKE_ROLLER_CAN_ID = 17;
    public static final int SHOOTER_FEEDER_CAN_ID = 18;
    public static final int HOPPER_TRANSFER_CAN_ID = 19;
    public static final int CHAIN_CLIMBER_FOLLOWER_MOTOR_CAN_ID = 20; 
    public static final int TURRET_CAN_ID = 21;

    public static final int TURRET_SMALL_CANCODER_ID = 5;
    public static final int TURRET_LARGE_CANCODER_ID = 6;

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

    // Feature flag: enable simulation in the classes
    // Can turn this off for competition to save a tiny bit of speed
    public static final boolean SIMULATION_SUPPORT = true;
}
