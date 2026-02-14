// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Intake extends SubsystemBase {

  IntakePivot m_intakePivot;
  IntakeRoller m_intakeRoller;

  /** Creates a new Intake. */
  public Intake() {
    m_intakePivot = new IntakePivot();
    m_intakeRoller = new IntakeRoller();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command stowCommand() {
    return new InstantCommand(() -> m_intakeRoller.intake(), m_intakeRoller)
                .andThen(new InstantCommand(() -> m_intakePivot.setAngle(IntakePivot.STOW_POSITION), m_intakePivot))
                .andThen(new WaitUntilCommand(m_intakePivot::onTarget))
                .andThen(new InstantCommand(() -> m_intakePivot.setAngle(IntakePivot.STOW_POSITION, IntakePivot.SlotNumber.HOLD), m_intakePivot))
                .andThen(new InstantCommand(m_intakeRoller::stop, m_intakeRoller));
  }

  public Command deployCommand() {
    return m_intakePivot.deployCommand();
  }

  public Command runRollers() {
    return new InstantCommand(m_intakeRoller::intake, m_intakeRoller);
  }

  public Command stopRollers() {
    return new InstantCommand(m_intakeRoller::intake, m_intakeRoller);
  }
  
  public IntakeRoller getRoller() {
    return m_intakeRoller;
  }

  public IntakePivot getPivot() {
    return m_intakePivot;
  }
}
