// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Turret;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
// import frc.robot.Constants;

// public class turretDriverControl extends CommandBase {
//   public static double posDriverControl;
//   /** Creates a new driverControl. */
//   public turretDriverControl() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     //addRequirements(RobotContainer.m_turret);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     posDriverControl = RobotContainer.m_turret.getPosition();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     posDriverControl = RobotContainer.m_turret.getPosition();

//     if(RobotContainer.joyStick.getPOV() == 270 && Constants.TURRET_TARGET_POSITION >= Constants.TURRET_LEFT_SOFT_LIMIT) {
//       Constants.TURRET_TARGET_POSITION = posDriverControl - 750;
//       RobotContainer.m_turret.turretCanTurn(Constants.TURRET_TARGET_POSITION);
//     }
//     else if(RobotContainer.joyStick.getPOV() == 90 && Constants.TURRET_TARGET_POSITION <= Constants.TURRET_RIGHT_SOFT_LIMIT) {
//       Constants.TURRET_TARGET_POSITION = posDriverControl + 750;
//       RobotContainer.m_turret.turretCanTurn(Constants.TURRET_TARGET_POSITION);
//     }
//     else {
//       RobotContainer.m_turret.stopTurret();
//     }
//   }


//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     RobotContainer.m_turret.stopTurret();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
