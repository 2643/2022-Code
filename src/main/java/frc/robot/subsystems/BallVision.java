// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallVision extends SubsystemBase {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
  public BallVision() {}

  public void sendColor()
  {
      Alliance color = DriverStation.getAlliance();
      String colorString;
      if(color == Alliance.Red)
      {
        colorString = "Red";
      }
      else if(color == Alliance.Blue)
      {
        colorString = "Blue";
      }
      else
      {
        colorString = "Can't find color";
      }
      table.getEntry("color").setString(colorString);
  }
  public double getCenterOfBall()
  {
    return table.getEntry("center").getDouble(-1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
