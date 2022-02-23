// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class climbUp extends CommandBase {
  /** Creates a new climbUp. */
  double pos;
  double posR;
  double posL;
  double currentPos;
  double diffErr;
  double gain;
  public climbUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gain=10; //test for gain later
    posR = RobotContainer.m_climber.getPositionR();
    posL = RobotContainer.m_climber.getPositionL();
    currentPos=(posL+posR)/2;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
    {
    diffErr=(posL-posR)*gain;
    pos=currentPos+20-diffErr;
    RobotContainer.m_climber.movePositionR(pos);
    RobotContainer.m_climber.movePositionL(pos);
    //System.out.println("Pos: " + pos + "PositionCurrent: " + RobotContainer.m_climber.getPosition());
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    double endPosR = RobotContainer.m_climber.getPositionR();
    double endPosL = RobotContainer.m_climber.getPositionL();
    RobotContainer.m_climber.movePositionR(endPosR);
    RobotContainer.m_climber.movePositionL(endPosL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
