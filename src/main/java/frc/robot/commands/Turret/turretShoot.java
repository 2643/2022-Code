// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class turretShoot extends CommandBase {
  /** Creates a new turretShoot. */
  boolean turretReady;
  double error;

  public turretShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    turretReady = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    error = (double)Constants.visionTable.getEntry("Degrees").getNumber(Constants.defaultVisionTurretError);
    if(error <= 1 || error >= -1) {
      turretReady = true;
    }
    else if(error >= 1) {
      RobotContainer.m_turret.turretCanTurn(RobotContainer.m_turret.getPosition() - 20);
    }
    else if(error <= -1) {
      RobotContainer.m_turret.turretCanTurn(RobotContainer.m_turret.getPosition() + 20);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(turretReady) {
      return true;
    }
    else {
      return false;
    }
  }
}
