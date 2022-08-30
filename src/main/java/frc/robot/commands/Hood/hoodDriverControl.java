package frc.robot.commands.Hood;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class hoodDriverControl extends CommandBase {
  /** Creates a new Hoodlmao. */
  private static double targetPos = 0;
  public hoodDriverControl() {
    addRequirements(RobotContainer.m_hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.joyStick.getPOV() == 0 && targetPos <= Constants.HOOD_UP_SOFT_LIMIT){
        targetPos = (RobotContainer.m_hood.getPosition() + 1000);
        RobotContainer.m_hood.hoodPID(targetPos);
    }
    else if(RobotContainer.joyStick.getPOV() == 180 && targetPos >= Constants.HOOD_DOWN_SOFT_LIMIT){
        targetPos = (RobotContainer.m_hood.getPosition() - 1000);
        RobotContainer.m_hood.hoodPID(targetPos);
    }
    else{
      RobotContainer.m_hood.stopMove();
      //RobotContainer.m_hood.hoodPID(RobotContainer.m_hood.getPosition());
    }
    //ShuffleboardTab HoodTab = Shuffleboard.getTab("HoodTab");
    //NetworkTableEntry targetHood = HoodTab.add("Target", 0).getEntry();
    //targetHood.setDouble(targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_hood.stopMove();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
