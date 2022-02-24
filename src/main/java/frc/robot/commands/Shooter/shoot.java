// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class shoot extends CommandBase {
  double motorSpeed;
  /** Creates a new shoot. */
  public shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    motorSpeed = (double)Constants.visionTable.getEntry("Distance").getNumber(Constants.defaultDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    motorSpeed = motorSpeed*5;
    RobotContainer.m_shooter.setSpeed(motorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.m_shooter.getVelocity() > 0){
      return true;
    }
    else{
      return false;
    }
  }
}