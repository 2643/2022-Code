// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimbUp extends CommandBase {
  /** Creates a new ClimbUp. */
  public ClimbUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hangclimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
  
  public void setMotorSpeed(double speed){
    RobotContainer.m_drivetrain.setMotorSpeed(speed);
  }

  public void speedClimb (double speed) {
    RobotContainer.hangclimb.speedClimb(speed);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.hangclimb.potPosition()<Constants.abovsecondrung){
      speedClimb(0); //test for a good speed
    }
    else if (RobotContainer.hangclimb.potPosition()==Constants.secondrung)

      setMotorSpeed(0);// move the robot a bit so that it can align with the rungs
      speedClimb(-10); // test for a good inverted speed
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
