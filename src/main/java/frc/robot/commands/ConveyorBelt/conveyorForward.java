// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ConveyorBelt;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.m_conveyorBelt;

public class conveyorForward extends CommandBase {
  /** Creates a new forwardConv. */
  public conveyorForward() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(RobotContainer.m_conveyorBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_conveyorBelt.setSpeed(Constants.CONVEYOR_MOTOR_SPEED);
    //RobotContainer.m_conveyorBelt.shootPrep();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_conveyorBelt.setSpeed(0);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
