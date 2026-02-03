// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberArms;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndGameDoubleArmClimb extends Command {
  ClimberArms m_climber;

  public ClimbState m_climbState = ClimbState.Beginning;

  public static final int ARM_FULLY_EXTENDED_LENGTH_INCHES = 26;

  private enum ClimbState {
    Beginning,
    LevelOne,
    LevelTwo,
    LevelThree,
    GoToInitialPosition
  }

  /** Creates a new Climb. */
  public EndGameDoubleArmClimb(ClimberArms climberArms) {
    m_climber = climberArms;
    addRequirements(m_climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbState = ClimbState.GoToInitialPosition;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (m_climbState) {
      case GoToInitialPosition:
        m_climber.setPosition(ARM_FULLY_EXTENDED_LENGTH_INCHES, ClimberArms.MotorSelection.LEFT, false);
        m_climber.setPosition(ARM_FULLY_EXTENDED_LENGTH_INCHES, ClimberArms.MotorSelection.RIGHT, false);
        break;

      case Beginning:

        if (m_climber.onTarget(ClimberArms.MotorSelection.LEFT) && m_climber.onTarget(ClimberArms.MotorSelection.RIGHT)) {

          m_climbState = ClimbState.LevelOne;
          m_climber.setPosition(0, ClimberArms.MotorSelection.RIGHT, false);
        }

        break;
      case LevelOne:
        
        if (m_climber.onTarget(ClimberArms.MotorSelection.RIGHT)) {
          
          m_climbState = ClimbState.LevelTwo;
          m_climber.setPosition(0, ClimberArms.MotorSelection.LEFT, false);
          m_climber.setPosition(26, ClimberArms.MotorSelection.RIGHT, false);
        }

        break;
      case LevelTwo:
      
        if (m_climber.onTarget(ClimberArms.MotorSelection.LEFT) && m_climber.onTarget(ClimberArms.MotorSelection.RIGHT)) {
          
          m_climbState = ClimbState.LevelThree;
          m_climber.setPosition(0, ClimberArms.MotorSelection.RIGHT, false);
        }

        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_climber.onTarget(ClimberArms.MotorSelection.RIGHT)) {
      return true;
    }
    return false;
  }
}
