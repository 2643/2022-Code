// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MovePosition extends CommandBase {
  private double robotPosition;

  /** Creates a new MovePosition. */
  public MovePosition(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    robotPosition = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_drivetrain.resetMotorEncoders();

    RobotContainer.m_drivetrain.setMotorPosition(robotPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // logic based on 2021-code MoveForward.java
    if (Math.abs(RobotContainer.m_drivetrain.getLeftMotorPosition()) <= Constants.DRIVETRAIN_ALLOWED_ERROR 
      && Math.abs(RobotContainer.m_drivetrain.getLeftMotorPosition()) >= Constants.DRIVETRAIN_ALLOWED_ERROR) {
        if (Math.abs(RobotContainer.m_drivetrain.getRightMotorPosition()) <= Constants.DRIVETRAIN_ALLOWED_ERROR 
      && Math.abs(RobotContainer.m_drivetrain.getRightMotorPosition()) >= Constants.DRIVETRAIN_ALLOWED_ERROR) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
}
