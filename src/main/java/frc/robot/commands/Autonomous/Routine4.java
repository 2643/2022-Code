// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Moves back, shots the ball, gets the second ball, shoots ball in the hub
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
//import frc.robot.commands.hoodcm;
import frc.robot.commands.PigeonTwo.turnRobot;
//import frc.robot.commands.Shooter.shoot;
import frc.robot.commands.ConveyorBelt.conveyorForward;
import frc.robot.commands.Drivetrain.MovePosition;
import frc.robot.commands.Intake.moveIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Routine4 extends SequentialCommandGroup {
  /** Creates a new Routine4. */
  public Routine4(double delaySeconds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new WaitCommand(delaySeconds),
                new conveyorForward().raceWith(new WaitCommand(2)),
                //new shoot(Constants.AUTONOMOUS_SHOOTER_SPEED).raceWith(new WaitCommand(3)), 
                new MovePosition(Constants.ROUTINE_THREE_MOVE_POSITION_AUTONOMOUS_TO_GET_POINTS), 
                new turnRobot(180), 
                new ParallelCommandGroup(new MovePosition(-Constants.ROUTINE_THREE_MOVE_POSITION_TO_GET_BALL), 
                                         new moveIntake().raceWith(new WaitCommand(3))), 
                new turnRobot(180)//, 
                //new shoot(Constants.AUTONOMOUS_SHOOTER_SPEED).raceWith(new WaitCommand(3))
                );
  }
}
