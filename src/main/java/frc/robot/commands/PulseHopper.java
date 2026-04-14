package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class PulseHopper extends Command {
    private static final double DEFAULT_PULSE_FORWARD_VOLTAGE = 5.0;
    private static final double DEFAULT_PULSE_REVERSE_VOLTAGE = -10.0;
    private static final double DEFAULT_PULSE_FORWARD_SEC = 0.35;
    private static final double DEFAULT_PULSE_REVERSE_SEC = 0.1;

    private static final String PULSE_FORWARD_VOLTAGE_KEY = "hopper/pulseForwardVoltage";
    private static final String PULSE_REVERSE_VOLTAGE_KEY = "hopper/pulseReverseVoltage";
    private static final String PULSE_FORWARD_SEC_KEY = "hopper/pulseForwardSec";
    private static final String PULSE_REVERSE_SEC_KEY = "hopper/pulseReverseSec";

    // Feature toggle dashboard keys.
    private static final String PULSE_GATE_ENABLED_KEY = "hopper/pulseGateEnabled";

    // Feature toggle defaults.
    private static final boolean DEFAULT_PULSE_GATE_ENABLED = false;

    private final Hopper m_hopper;
    private final BooleanSupplier m_canPulseSupplier;

    private boolean m_forwardPhase = true;
    private boolean m_pulseActive = false;
    private double m_phaseStartTimeSec = 0.0;

    public PulseHopper(Hopper hopper, BooleanSupplier canPulseSupplier) {
        m_hopper = hopper;
        m_canPulseSupplier = canPulseSupplier;

        SmartDashboard.setDefaultNumber(PULSE_FORWARD_VOLTAGE_KEY, DEFAULT_PULSE_FORWARD_VOLTAGE);
        SmartDashboard.setDefaultNumber(PULSE_REVERSE_VOLTAGE_KEY, DEFAULT_PULSE_REVERSE_VOLTAGE);
        SmartDashboard.setDefaultNumber(PULSE_FORWARD_SEC_KEY, DEFAULT_PULSE_FORWARD_SEC);
        SmartDashboard.setDefaultNumber(PULSE_REVERSE_SEC_KEY, DEFAULT_PULSE_REVERSE_SEC);

        // Feature toggles live together here so they are easy to find.
        SmartDashboard.setDefaultBoolean(PULSE_GATE_ENABLED_KEY, DEFAULT_PULSE_GATE_ENABLED);

        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        resetPulsePattern();
        m_hopper.stop();
    }

    @Override
    public void execute() {
        boolean canPulse = isPulseGateEnabled() && m_canPulseSupplier.getAsBoolean();
        if (canPulse) {
            runPulsePattern();
        } else {
            resetPulsePattern();
            m_hopper.feed();
        }

        SmartDashboard.putBoolean("hopper/pulseAllowed", canPulse);
        SmartDashboard.putBoolean("hopper/pulseActive", m_pulseActive);
        SmartDashboard.putBoolean("hopper/feeding", !canPulse);
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stop();
        resetPulsePattern();
        SmartDashboard.putBoolean("hopper/pulseAllowed", false);
        SmartDashboard.putBoolean("hopper/pulseActive", false);
        SmartDashboard.putBoolean("hopper/feeding", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void runPulsePattern() {
        double now = Timer.getFPGATimestamp();

        if (!m_pulseActive) {
            m_pulseActive = true;
            m_forwardPhase = true;
            m_phaseStartTimeSec = now;
            m_hopper.setVoltage(getPulseForwardVoltage());
            return;
        }

        if (m_forwardPhase && now - m_phaseStartTimeSec >= getPulseForwardSec()) {
            m_forwardPhase = false;
            m_phaseStartTimeSec = now;
            m_hopper.setVoltage(getPulseReverseVoltage());
        } else if (!m_forwardPhase && now - m_phaseStartTimeSec >= getPulseReverseSec()) {
            m_forwardPhase = true;
            m_phaseStartTimeSec = now;
            m_hopper.setVoltage(getPulseForwardVoltage());
        }
    }

    private void resetPulsePattern() {
        m_pulseActive = false;
        m_forwardPhase = true;
        m_phaseStartTimeSec = Timer.getFPGATimestamp();
    }

    private double getPulseForwardVoltage() {
        return SmartDashboard.getNumber(PULSE_FORWARD_VOLTAGE_KEY, DEFAULT_PULSE_FORWARD_VOLTAGE);
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

    private static boolean isPulseGateEnabled() {
        return SmartDashboard.getBoolean(PULSE_GATE_ENABLED_KEY, DEFAULT_PULSE_GATE_ENABLED);
    }
}
