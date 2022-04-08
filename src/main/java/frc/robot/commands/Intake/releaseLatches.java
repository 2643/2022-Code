// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class releaseLatches extends CommandBase {
  public double unlatchDegrees;

  public static Servo rightServo = new Servo(Constants.INTAKE_RIGHT_SERVO_CHANNEL);
  public static Servo leftServo = new Servo(Constants.INTAKE_LEFT_SERVO_CHANNEL);

  /** Creates a new releaseLatches. */
  public releaseLatches(double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
    unlatchDegrees = degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightServo.setAngle(unlatchDegrees);
    leftServo.setAngle(unlatchDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}