
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class shoot extends CommandBase {

  private double rpm;
  private double startTime;

  /** Creates a new shoot. */
  public shoot(double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooter);
    addRequirements(RobotContainer.conveyorBelt);
    rpm = velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.conveyorBelt.shootPrep();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double execStart = Timer.getFPGATimestamp() - startTime;

    RobotContainer.m_shooter.setVelSpeed(rpm);
    System.out.println(RobotContainer.m_shooter.getVelocity());

    if (RobotContainer.m_shooter.getVelocity() >= rpm) {
      RobotContainer.conveyorBelt.setSpeed(0.9);
    }
    // if (Timer.getFPGATimestamp() < (execStart + 2)) {
    //   // RobotContainer.conveyorBelt.setSpeed(Constants.convRevMotorSpeed);
    // } else if (Timer.getFPGATimestamp() < (startTime + 2.5)) {
    //   System.out.println(Timer.getFPGATimestamp());
    //   System.out.println(execStart);
    //   // RobotContainer.m_shooter.setVelSpeed(2500);
    //   RobotContainer.conveyorBelt.setSpeed(0.9);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_shooter.setSpeed(0);
    RobotContainer.conveyorBelt.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}