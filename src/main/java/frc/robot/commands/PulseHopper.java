package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;

public class PulseHopper extends Command {
    private static final double DEFAULT_STARTUP_REVERSE_VOLTAGE = -11.0;
    private static final double DEFAULT_STARTUP_REVERSE_TIMEOUT_SEC = 0.45;
    private static final double DEFAULT_PULSE_FORWARD_VOLTAGE = 6.0;
    private static final double DEFAULT_PULSE_REVERSE_VOLTAGE = -10.0;
    private static final double DEFAULT_PULSE_FORWARD_SEC = 0.4;
    private static final double DEFAULT_PULSE_REVERSE_SEC = 0.2;

    private static final String STARTUP_REVERSE_VOLTAGE_KEY = "hopper/startupReverseVoltage";
    private static final String STARTUP_REVERSE_TIMEOUT_KEY = "hopper/startupReverseTimeoutSec";
    private static final String PULSE_FORWARD_VOLTAGE_KEY = "hopper/pulseForwardVoltage";
    private static final String PULSE_REVERSE_VOLTAGE_KEY = "hopper/pulseReverseVoltage";
    private static final String PULSE_FORWARD_SEC_KEY = "hopper/pulseForwardSec";
    private static final String PULSE_REVERSE_SEC_KEY = "hopper/pulseReverseSec";

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

        SmartDashboard.setDefaultNumber(STARTUP_REVERSE_VOLTAGE_KEY, DEFAULT_STARTUP_REVERSE_VOLTAGE);
        SmartDashboard.setDefaultNumber(STARTUP_REVERSE_TIMEOUT_KEY, DEFAULT_STARTUP_REVERSE_TIMEOUT_SEC);
        SmartDashboard.setDefaultNumber(PULSE_FORWARD_VOLTAGE_KEY, DEFAULT_PULSE_FORWARD_VOLTAGE);
        SmartDashboard.setDefaultNumber(PULSE_REVERSE_VOLTAGE_KEY, DEFAULT_PULSE_REVERSE_VOLTAGE);
        SmartDashboard.setDefaultNumber(PULSE_FORWARD_SEC_KEY, DEFAULT_PULSE_FORWARD_SEC);
        SmartDashboard.setDefaultNumber(PULSE_REVERSE_SEC_KEY, DEFAULT_PULSE_REVERSE_SEC);

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
            m_hopper.setVoltage(getStartupReverseVoltage());

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
        SmartDashboard.putBoolean("hopper/shooterLatched", m_shooterOnTarget);
        SmartDashboard.putBoolean("hopper/startupReverseActive", m_startupReverseActive);
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stop();
        m_startupReverseTimer.stop();
        SmartDashboard.putBoolean("hopper/pulseActive", false);
        SmartDashboard.putBoolean("hopper/shooterLatched", false);
        SmartDashboard.putBoolean("hopper/startupReverseActive", false);
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
            m_hopper.setVoltage(getPulseForwardVoltage());
            return;
        }

        if (m_pulsingForward && now - m_lastPulsePhaseTimeSec >= getPulseForwardSec()) {
            m_pulsingForward = false;
            m_lastPulsePhaseTimeSec = now;
            m_hopper.setVoltage(getPulseReverseVoltage());
        } else if (!m_pulsingForward && now - m_lastPulsePhaseTimeSec >= getPulseReverseSec()) {
            m_pulsingForward = true;
            m_lastPulsePhaseTimeSec = now;
            m_hopper.setVoltage(getPulseForwardVoltage());
        }
    }

    private double getPulseForwardVoltage() {
        return SmartDashboard.getNumber(PULSE_FORWARD_VOLTAGE_KEY, DEFAULT_PULSE_FORWARD_VOLTAGE);
    }

    private double getStartupReverseVoltage() {
        return SmartDashboard.getNumber(STARTUP_REVERSE_VOLTAGE_KEY, DEFAULT_STARTUP_REVERSE_VOLTAGE);
    }

    private double getStartupReverseTimeoutSec() {
        return Math.max(0.0, SmartDashboard.getNumber(STARTUP_REVERSE_TIMEOUT_KEY, DEFAULT_STARTUP_REVERSE_TIMEOUT_SEC));
    }

    private double getPulseReverseVoltage() {
        return SmartDashboard.getNumber(PULSE_REVERSE_VOLTAGE_KEY, DEFAULT_PULSE_REVERSE_VOLTAGE);
    }

    private double getPulseForwardSec() {
        return Math.max(0.0, SmartDashboard.getNumber(PULSE_FORWARD_SEC_KEY, DEFAULT_PULSE_FORWARD_SEC));
    }

    private double getPulseReverseSec() {
        return Math.max(0.0, SmartDashboard.getNumber(PULSE_REVERSE_SEC_KEY, DEFAULT_PULSE_REVERSE_SEC));
    }
}
