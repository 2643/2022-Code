// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PigeonTwo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class turnRobot extends CommandBase {
  static double turnDegrees;
  /** Creates a new turnRobot. */
  public turnRobot(double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_gyro);
    addRequirements(RobotContainer.m_drivetrain);
    turnDegrees = degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentAngle = RobotContainer.m_gyro.gyroAngle();
    turnDegrees += currentAngle;
    RobotContainer.m_gyro.turnClockwiseDegrees(turnDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.setMotorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.round(RobotContainer.m_gyro.gyroAngle()) == turnDegrees){
      return true;
    } else{
      return false;
    }
  }
}
