// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Delayed for some period of time, moves back, shots the ball, gets the second ball, shoots ball in the hub

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.Constants;
// import frc.robot.commands.hoodcm;
import frc.robot.commands.PigeonTwo.turnRobot;
import frc.robot.Constants;
import frc.robot.commands.ConveyorBelt.autoConveyorIntake;
import frc.robot.commands.Drivetrain.MovePosition;
import frc.robot.commands.Intake.moveIntake;
//import frc.robot.commands.Shooter.shoot;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Routine2 extends SequentialCommandGroup {
  /** Creates a new Rountine2. */
  public Routine2(double delay) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());-
    //addCommands(new turnRobot(180));
    addCommands(new WaitCommand(delay), new MovePosition(-2048*4.3*Constants.DRIVETRAIN_GEARBOX_RATIO), new ParallelCommandGroup(new moveIntake().raceWith(new WaitCommand(4))), new autoConveyorIntake().raceWith(new WaitCommand(2)), new MovePosition(2048*1.1*Constants.DRIVETRAIN_GEARBOX_RATIO), new turnRobot(176)/*new shoot(0.5).raceWith(new WaitCommand(4)), new* shoot(0.6).raceWith(new WaitCommand(4))*/); 
  }  
}

// Facing away from the hub