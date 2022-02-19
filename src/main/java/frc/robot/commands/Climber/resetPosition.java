// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class resetPosition extends CommandBase {
  /** Creates a new ClimbUp. */
  public resetPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
  
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!RobotContainer.m_climber.limitswitch())
    {
        RobotContainer.m_climber.setPosition(0);
    }
    else
    {
        RobotContainer.m_climber.movePosition(-100000);
    }
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!RobotContainer.m_climber.limitswitch())
    {
        return true;
    }
    else
    {
        return false;
    }
  }
}