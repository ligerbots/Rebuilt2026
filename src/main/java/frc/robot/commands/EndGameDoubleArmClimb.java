// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberArms;
import frc.robot.subsystems.ClimberArms.ClimbState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndGameDoubleArmClimb extends Command {
  ClimberArms m_climber;

  private ClimbState m_goalState;

  private static final int ARM_FULLY_EXTENDED_LENGTH_INCHES = 26;
  private static final int ARM_FULLY_RETRACTED_LENGTH_INCHES = 0;

  /** Creates a new Climb. */
  public EndGameDoubleArmClimb(ClimberArms climberArms, ClimbState goalState) {
    m_goalState = goalState;
    m_climber = climberArms;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (m_climber.getClimbState()) {
      case Stowed:
        m_climber.setDistance(ARM_FULLY_EXTENDED_LENGTH_INCHES, ClimberArms.MotorSelection.LEFT, false);
        m_climber.setDistance(ARM_FULLY_EXTENDED_LENGTH_INCHES, ClimberArms.MotorSelection.RIGHT, false);

        m_climber.setClimbState(ClimbState.Extended);
        break;
      
      case Extended:
        // Paused, waiting for command to start climb
        break;

      case LevelOne:

        m_climber.setDistance(ARM_FULLY_RETRACTED_LENGTH_INCHES, ClimberArms.MotorSelection.RIGHT, false);
       if (m_climber.onTarget(ClimberArms.MotorSelection.RIGHT)) {
         
          m_climber.setClimbState(ClimbState.LevelTwo);
        }

        break;
      
      case LevelTwo:
        
        m_climber.setDistance(ARM_FULLY_RETRACTED_LENGTH_INCHES, ClimberArms.MotorSelection.LEFT, false);
        m_climber.setDistance(ARM_FULLY_EXTENDED_LENGTH_INCHES, ClimberArms.MotorSelection.RIGHT, false);

        if (m_climber.onTarget(ClimberArms.MotorSelection.RIGHT) && m_climber.onTarget(ClimberArms.MotorSelection.LEFT)) {
          m_climber.setClimbState(ClimbState.LevelThree);
        }

        break;
      
      case LevelThree:
        m_climber.setDistance(ARM_FULLY_RETRACTED_LENGTH_INCHES, ClimberArms.MotorSelection.RIGHT, false);
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_goalState == m_climber.getClimbState() && m_climber.onTarget(ClimberArms.MotorSelection.RIGHT) && m_climber.onTarget(ClimberArms.MotorSelection.LEFT);
  }
}
