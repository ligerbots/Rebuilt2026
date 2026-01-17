// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  public Hood() {}

  private Rotation2d goalAngle = Rotation2d.fromDegrees(0.0);

  // CTRE Talon FX configuration for the Kraken X44 (Talon FX compatible)
  private final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(Rotation2d angle) {
    goalAngle = angle;
  }

  public Rotation2d readAngle() {

    return goalAngle; // example
  }


}
