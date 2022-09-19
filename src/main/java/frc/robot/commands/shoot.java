
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class shoot extends CommandBase {

  double percent;

  /** Creates a new shoot. */
  public shoot(double m_percent) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooter);
    //addRequirements(RobotContainer.conveyorBelt);
    percent = m_percent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.conveyorBelt.shootPrep();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_shooter.setSpeed(Constants.SHOOTER_UP_SPEED);
    Timer.delay(1.5);
    RobotContainer.m_shooter.setSpeed(percent);
    Timer.delay(2);
    //RobotContainer.conveyorBelt.shootPulse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}