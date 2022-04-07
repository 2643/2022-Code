// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.MovePosition;
import frc.robot.commands.Shooter.shoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Routine1 extends SequentialCommandGroup {
  //private final Timer m_timer = new Timer();
  /** Creates a new Rountine3. */
  public Routine1(double delay) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new WaitCommand(delay), new MovePosition(Constants.MOVE_POSITION_AUTONOMOUS_TO_GET_POINTS), new shoot(Constants.FAR_SHOOTER_SPEED).raceWith(new WaitCommand(4)));
    }
  }