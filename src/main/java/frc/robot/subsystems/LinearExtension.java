// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearExtension extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // define all constants here
  private final TalonFX m_motor; //import the talonfx stuff in at top of code

  private static final double MAX_LENGTH;
  private static final double MIN_LENGTH;


  public LinearExtension() {

    m_motor = new TalonFX(); //add in motor id constant

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
