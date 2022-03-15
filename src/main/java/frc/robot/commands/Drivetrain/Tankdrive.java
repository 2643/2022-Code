// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Tankdrive extends CommandBase {
  /** Creates a new Tankdrive. */
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private boolean finished = false;
  private int loopcounter = 0;
  private double twentyPercentSetpoint = Constants.TANKDRIVE_SETPOINT/5.0;

  public Tankdrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS)) < Constants.JOYSITCK_DEADBAND && leftSpeed == 0) {
      if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS)) < Constants.JOYSITCK_DEADBAND && rightSpeed == 0) {
        finished = true;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left_input = RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS);
    double right_input = RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS);
    int multiplier = controlToMultiplier(RobotContainer.opBoard.getRawAxis(2));
    int slewMultiplier = (6-multiplier);
    
    if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS)) < Constants.JOYSITCK_DEADBAND) {
      left_input = 0;
    }
    if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS)) < Constants.JOYSITCK_DEADBAND) {
      right_input = 0;
    }
    if (multiplier != -1) {
      if (leftSpeed - left_input > Constants.TANKDRIVE_SLEW_RATE*slewMultiplier) {
        leftSpeed -= Constants.TANKDRIVE_SLEW_RATE*slewMultiplier;
      } else if (leftSpeed - left_input < -Constants.TANKDRIVE_SLEW_RATE*slewMultiplier) {
        leftSpeed += Constants.TANKDRIVE_SLEW_RATE*slewMultiplier;
      } else {
        leftSpeed = left_input;
      }

      if (rightSpeed - right_input > Constants.TANKDRIVE_SLEW_RATE*slewMultiplier) {
        rightSpeed -= Constants.TANKDRIVE_SLEW_RATE*slewMultiplier;
      } else if (rightSpeed - right_input < -Constants.TANKDRIVE_SLEW_RATE*slewMultiplier) {
        rightSpeed += Constants.TANKDRIVE_SLEW_RATE*slewMultiplier;
      } else {
        rightSpeed = right_input;
      }

      RobotContainer.m_drivetrain.setLeftMotorVelocity(leftSpeed*multiplier*twentyPercentSetpoint);
      RobotContainer.m_drivetrain.setRightMotorVelocity(rightSpeed*multiplier*twentyPercentSetpoint);

      // if (++loopcounter % 5 == 0) {
      //   System.out.println();
      // }
    } else {
      RobotContainer.m_drivetrain.setLeftMotorPercentOutput(left_input);
      RobotContainer.m_drivetrain.setRightMotorPercentOutput(right_input);
    }
  }

  private int controlToMultiplier(double ctrlValue) {
    if (ctrlValue > 0.6) {
      return 1;
    } else if (ctrlValue > 0.4) {
      return 2;
    } else if (ctrlValue > 0.2) {
      return 3;
    } else if (ctrlValue > 0) {
      return 4;
    } else if (ctrlValue > -0.5) {
      return 5;
    } else {
      return -1;
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
