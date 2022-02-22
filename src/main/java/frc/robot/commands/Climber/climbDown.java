// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class climbDown extends CommandBase {
  /** Creates a new climbUpDown. */
  double position;
  double pos = RobotContainer.m_climber.getPosition();
  public climbDown() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    pos = RobotContainer.m_climber.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    position -= RobotContainer.m_climber.getPosition();
    pos -= 100;
    RobotContainer.m_climber.movePosition(pos);
    System.out.println("Pos: " + pos + "PositionCurrent: " + RobotContainer.m_climber.getPosition());
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    double endPos = RobotContainer.m_climber.getPosition();
    RobotContainer.m_climber.movePosition(endPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
