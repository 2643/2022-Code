// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DifferentialDrive extends CommandBase {
  private static double speed;
  private static double rotation;
  private static boolean rotateInPlace;
  /** Creates a new DifferentialDrive. */
  public DifferentialDrive(double m_speed, double m_rotation, boolean m_rotateInPlace) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    speed = m_speed;
    rotation = m_rotation;
    rotateInPlace = m_rotateInPlace;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.m_drivetrain.m_drive.curvatureDrive(speed, rotation, rotateInPlace);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.driveButton.get()){
      return false;
    }
    else{
      return true;
    }
  }
}
