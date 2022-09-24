// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoShoot;

// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class autoShoot extends CommandBase {
  double distance;
  static double shooterRPM;
  /** Creates a new autoShoot. */
  public autoShoot(double distance) {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(RobotContainer.m_shooter);
    addRequirements(RobotContainer.m_turret);
    addRequirements(RobotContainer.m_conveyorBelt);
    //In inches
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.shootButton.get()){
      shooterRPM = 6.317 * (distance) + 1470;
      RobotContainer.m_shooter.setVelSpeed(shooterRPM);
       
      if(RobotContainer.m_shooter.getVelocity() >= (shooterRPM+20) || RobotContainer.m_shooter.getVelocity() <= (shooterRPM-16)){
        RobotContainer.m_conveyorBelt.setSpeed(0);
      } else if (RobotContainer.m_shooter.getVelocity() >= (shooterRPM-15)) {
        RobotContainer.m_conveyorBelt.setSpeed(0.9);
      } else {
        RobotContainer.m_conveyorBelt.setSpeed(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_shooter.setVelSpeed(0);
    RobotContainer.m_conveyorBelt.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
