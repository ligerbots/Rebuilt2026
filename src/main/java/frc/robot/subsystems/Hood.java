// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  public Hood() {}

  Rotation2d goalAngle;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(Rotation2d angle) {
    
  }

  public Rotation2d readAngle() {

    return new Rotation2d(2.1,2.3); // example
  }

  
}
