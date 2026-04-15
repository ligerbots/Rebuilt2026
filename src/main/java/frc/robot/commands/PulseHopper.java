package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.utilities.RobotLog;

public class PulseHopper extends Command {
    private static final double STARTUP_REVERSE_SPEED_RPM = -3000.0;
    private static final double STARTUP_REVERSE_TIMEOUT_SEC = 0.125;
    private static final double PULSE_FORWARD_SEC = 0.4;
    private static final double PULSE_REVERSE_SEC = 0.2;

    private final Hopper m_hopper;
    private final Shooter m_shooter;

    private boolean m_shooterOnTarget = false;
    private boolean m_startupReverseActive = true;
    private boolean m_pulsingForward = true;
    private boolean m_isPulsing = false;
    private double m_lastPulsePhaseTimeSec = 0.0;
    private final Timer m_startupReverseTimer = new Timer();

    public PulseHopper(Hopper hopper, Shooter shooter, Turret turret) {
        m_hopper = hopper;
        m_shooter = shooter;

        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        m_shooterOnTarget = false;
        m_startupReverseActive = true;
        m_isPulsing = false;
        m_pulsingForward = true;
        m_lastPulsePhaseTimeSec = Timer.getFPGATimestamp();
        m_startupReverseTimer.restart();
    }

    @Override
    public void execute() {
        if (!m_shooterOnTarget && m_shooter.onTarget()) {
            m_shooterOnTarget = true;
        }

        if (m_startupReverseActive) {
            m_hopper.setRPM(getStartupReverseSpeed());

            boolean shotDetected = m_shooterOnTarget && m_shooter.getFlywheel().isShotDetected();
            boolean startupTimedOut = m_startupReverseTimer.hasElapsed(getStartupReverseTimeoutSec());
            if (shotDetected || startupTimedOut) {
                m_startupReverseActive = false;
                m_isPulsing = false;
                m_pulsingForward = true;
                m_lastPulsePhaseTimeSec = Timer.getFPGATimestamp();
            }
        } else if (m_shooter.getFlywheel().isCurrentJamDetected()) {  
            runPulseCycle();
        } else {
            m_isPulsing = false;
            m_hopper.feed();
        }

        SmartDashboard.putBoolean("hopper/pulseActive", m_isPulsing);
        RobotLog.log("hopper/shooterLatched", m_shooterOnTarget);
        RobotLog.log("hopper/startupReverseActive", m_startupReverseActive);
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stop();
        m_startupReverseTimer.stop();
        SmartDashboard.putBoolean("hopper/pulseActive", false);
        RobotLog.log("hopper/shooterLatched", false);
        RobotLog.log("hopper/startupReverseActive", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void runPulseCycle() {
        double now = Timer.getFPGATimestamp();

        if (!m_isPulsing) {
            m_isPulsing = true;
            m_pulsingForward = true;
            m_lastPulsePhaseTimeSec = now;
            m_hopper.reverse();
            return;
        }

        double timeSinceLastPulsePhase = now - m_lastPulsePhaseTimeSec;
        if (m_pulsingForward && timeSinceLastPulsePhase >= getPulseForwardSec()) {
            m_pulsingForward = false;
            m_lastPulsePhaseTimeSec = now;
            m_hopper.reverse();
        } else if (!m_pulsingForward && timeSinceLastPulsePhase >= getPulseReverseSec()) {
            m_pulsingForward = true;
            m_lastPulsePhaseTimeSec = now;
            m_hopper.feed();
        }
    }

    private double getStartupReverseSpeed() {
        return STARTUP_REVERSE_SPEED_RPM;
    }

    private double getStartupReverseTimeoutSec() {
        return STARTUP_REVERSE_TIMEOUT_SEC;
    }

    private double getPulseForwardSec() {
        return PULSE_FORWARD_SEC;
    }

    private double getPulseReverseSec() {
        return PULSE_REVERSE_SEC;
    }
}
