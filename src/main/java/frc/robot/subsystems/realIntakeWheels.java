// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**integrate motors and tell them to run, when button is pressed, then brake!! */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;

public class realIntakeWheels extends SubsystemBase {
  private final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();  
  private final TalonFX m_motor;
  private static final double K_P = 3.0;
  private static final int CURRENT_LIMIT = 90;
  
  /** Creates a new realIntakeWheels. */
  public realIntakeWheels() {
    m_motor = new TalonFX(Constants.INTAKE_CAN_ID);
    Slot0Configs slot0configs = talonFXConfigs.Slot0;
    slot0configs.kP = K_P;  // start small!!!
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withStatorCurrentLimit(CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);

    // enable brake mode (after main config)
    m_motor.getConfigurator().apply(talonFXConfigs);
    m_motor.setNeutralMode(NeutralModeValue.Brake);

    MotorOutputConfigs m_motor = new MotorOutputConfigs();
    m_motor.Inverted = InvertedValue.Clockwise_Positive;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
