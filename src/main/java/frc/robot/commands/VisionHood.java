// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class VisionHood extends CommandBase {
  /** Creates a new VisionHood. */
  boolean hoodReady; 
  double error;
  double pos;
  double gain;
  public static double target;
  public VisionHood() {
    addRequirements(RobotContainer.cm_Hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = RobotContainer.cm_Hood.getPositionValue();
    hoodReady = false;
    error = ((double)Constants.visionTable.getEntry("Degree2").getNumber(Constants.HOOD_DEFAULT_VISION_ERROR));
    target = pos - error*gain;
    RobotContainer.cm_Hood.moveHood(target);
    hoodReady = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pos = RobotContainer.cm_Hood.getPositionValue();
    error = ((double)Constants.visionTable.getEntry("Degree2").getNumber(Constants.HOOD_DEFAULT_VISION_ERROR));
    System.out.println("Error: " + error + " Pos:" + RobotContainer.cm_Hood.getPositionValue() + " Ready: " + hoodReady + " Target:" + target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(hoodReady){
      return true;
    }
    else{
      return false;
    }
  }
}
