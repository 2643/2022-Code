// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Tankdrive extends CommandBase {
  /** Creates a new Tankdrive. */
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private boolean finished = false;

  public Tankdrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS)) < Constants.JOYSITCK_DEADBAND) {
      if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS)) < Constants.JOYSITCK_DEADBAND) {
        finished = true;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.percentOutputControl){
      if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS)) > Constants.JOYSITCK_DEADBAND) {
      
        if(Constants.slowMode == true){
          leftSpeed = Constants.SLOW_MODE_MULTIPLIER*(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS));
        } else {
          leftSpeed = (RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS));
        }
      }
      else{
        leftSpeed = 0;
      }
  
      if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS)) > Constants.JOYSITCK_DEADBAND) {
        if(Constants.slowMode == true){
          rightSpeed = Constants.SLOW_MODE_MULTIPLIER*(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS));
        } else {
          rightSpeed = (RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS));
        }
      }
      else {
        rightSpeed = 0;
      }
      
      RobotContainer.m_drivetrain.setLeftMotorPercentOutput(leftSpeed);
      RobotContainer.m_drivetrain.setRightMotorPercentOutput(rightSpeed);
    }
    else{
      if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS)) > Constants.JOYSITCK_DEADBAND) {
      
        if(Constants.slowMode == true){
          leftSpeed = Constants.SLOW_MODE_MULTIPLIER*(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS));
        } else {
          leftSpeed = (RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS));
        }
      }
      else{
        leftSpeed = 0;
      }
  
      if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS)) > Constants.JOYSITCK_DEADBAND) {
        if(Constants.slowMode == true){
          rightSpeed = Constants.SLOW_MODE_MULTIPLIER*(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS));
        } else {
          rightSpeed = (RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS));
        }
      }
      else {
        rightSpeed = 0;
      }
      
      RobotContainer.m_drivetrain.setLeftMotorVelocity(leftSpeed*44000);
      RobotContainer.m_drivetrain.setRightMotorVelocity(rightSpeed*44000);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.setMotorSpeed(0);
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
