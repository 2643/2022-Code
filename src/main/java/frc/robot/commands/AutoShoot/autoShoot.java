// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoShoot;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class autoShoot extends CommandBase {
  double distanceOrRPM;
  static double shooterRPM;
  boolean isRPM;
  /** Creates a new autoShoot. */
  public autoShoot(double distanceIn, boolean trueISRPM) {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(RobotContainer.m_shooter);
    //addRequirements(RobotContainer.m_turret);
    addRequirements(RobotContainer.m_conveyorBelt);
    //In inches
    isRPM = trueISRPM;
    distanceOrRPM = distanceIn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_conveyorBelt.setSpeed(-0.1);
    Timer.delay(0.2);
    RobotContainer.m_conveyorBelt.setSpeed(0);
    if(isRPM){
      shooterRPM = distanceOrRPM;
    } else {
      shooterRPM = 6.317 * (distanceOrRPM) + 1470;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(shooterRPM<=4000 && shooterRPM>0){
      RobotContainer.m_shooter.setVelSpeed(shooterRPM);
      if(RobotContainer.m_shooter.getVelocity() >= (shooterRPM+15) || RobotContainer.m_shooter.getVelocity() <= (shooterRPM-11)){
        RobotContainer.m_conveyorBelt.setSpeed(0);    
      } else if (RobotContainer.m_shooter.getVelocity() >= (shooterRPM-10)) {
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
