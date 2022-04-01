// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import edu.wpi.first.wpilibj.Timer;

// public class LowerIntake extends CommandBase {
//   /** Creates a new LowerIntake. */
//   Timer timer = new Timer();
//   public LowerIntake() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(RobotContainer.m_intake);
//   } 

//   public boolean hasPeriodPassed(int period) {
//     return true;
//   }

//   public void intakeMotor (double d) {
//     RobotContainer.m_intake.setSpeed(d);
//   }
 
//   public void start() {
//     // while(!hasPeriodPassed(1)) {
//       intakeMotor(0.5);
//       System.out.println("its running");
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     // timer.reset();
//     // timer.start();
//     start(); 
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   }

  
//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     Constants.verticalIntakeToggleVariable = false;
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
