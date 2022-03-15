// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Climber;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;

// public class resetPosition extends CommandBase {
//   private double upr = RobotContainer.m_climber.getPositionR() + 115000;
//   private double upl = RobotContainer.m_climber.getPositionL() + 115000;

//   private boolean zeroedPositionR = false;
//   private boolean zeroedPositionL = false;


//   /** Creates a new ClimbUp. */
//   public resetPosition() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(RobotContainer.m_climber);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     RobotContainer.m_climber.movePositionL(upl);
//     RobotContainer.m_climber.movePositionR(upr);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(!RobotContainer.m_climber.limitswitchR()) {
//       RobotContainer.m_climber.percentOutputControlResetR(0.0);
//       RobotContainer.m_climber.setPositionR(20000);
//       RobotContainer.m_climber.movePositionR(20000);
//     }
//     else if(upr-RobotContainer.m_climber.getPositionR() <= 750 & upr-RobotContainer.m_climber.getPositionR() >= -750) {
//       RobotContainer.m_climber.percentOutputControlResetR(-0.5);
//     }
//     if(!RobotContainer.m_climber.limitswitchL()) {
//       RobotContainer.m_climber.percentOutputControlResetL(0.0);
//       RobotContainer.m_climber.setPositionL(20000);
//       RobotContainer.m_climber.movePositionL(20000);
//     }
//     else if(upl-RobotContainer.m_climber.getPositionL() <= 750 & upl-RobotContainer.m_climber.getPositionL() >= -750) {
//       RobotContainer.m_climber.percentOutputControlResetL(-0.5);   
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     RobotContainer.m_climber.percentOutputControlResetR(0.0);
//     RobotContainer.m_climber.percentOutputControlResetL(0.0);
//     RobotContainer.m_climber.movePositionR(0);
//     RobotContainer.m_climber.movePositionL(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if(!RobotContainer.m_climber.limitswitchR() && !RobotContainer.m_climber.limitswitchL()) {
//       if(zeroedPositionL && zeroedPositionR){
//         return true;
//       }
//       return false;
//     }
//     return false;
//   }
// }