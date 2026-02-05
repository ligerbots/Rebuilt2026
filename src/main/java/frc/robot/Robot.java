// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand = null;
    private boolean m_prevIsRedAlliance = true;

    public static final String TESTBOT_SERIAL_NUMBER = "0313baff";  // TODO: real value?
    public static final String COMPBOT_SERIAL_NUMBER = "030fc268";

    public enum RobotType {
        TESTBOT, COMPBOT
    }
    // we want this to be static so that it is easy for subsystems to query the robot type
    private static RobotType m_robotType;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        // Disable the LiveWindow telemetry to lower the network load
        LiveWindow.disableAllTelemetry();

        // Enable local logging.
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        // Figure out which roboRio this is, so we know which version of the robot
        //   code to run.
        String serialNum = HALUtil.getSerialNumber();
        SmartDashboard.putString("rioSerialNumber", serialNum);
        if (serialNum.equals(TESTBOT_SERIAL_NUMBER)) {
            m_robotType = RobotType.TESTBOT;
        } else if (serialNum.equals(COMPBOT_SERIAL_NUMBER)) {
            m_robotType = RobotType.COMPBOT;
        } else {
            // default to the Test robot for now
            m_robotType = RobotType.COMPBOT;
        }
        SmartDashboard.putString("robotType", m_robotType.toString());

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        if (m_robotType == RobotType.TESTBOT) {
            m_robotContainer = new RobotContainerTestBot();
        } else {
            m_robotContainer = new RobotContainerCompBot();
        }
    }

    // Useful if a subsystem needs to know which chassis
    public static RobotType getRobotType() {
        return m_robotType;
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        boolean isRedAlliance = FieldConstants.isRedAlliance();
        Command newAuto = m_robotContainer.getAutonomousCommand();

        // don't change the initialPose unless the Auto or Alliance has changed
        // don't want to override the true pose on the field (as determined by the AprilTags)
        //
        // Note: use "==" to compare autos - checks if they are the same object
        if (isRedAlliance != m_prevIsRedAlliance || newAuto != m_autonomousCommand) {
            m_autonomousCommand = newAuto;
            m_prevIsRedAlliance = isRedAlliance;

            // drivetrain might be null when testing code. So check
            CommandSwerveDrivetrain driveTrain = m_robotContainer.getDriveTrain();
            if (driveTrain != null) driveTrain.setPose(m_robotContainer.getInitialPose());
        }
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
