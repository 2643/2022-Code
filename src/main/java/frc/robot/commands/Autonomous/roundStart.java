// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ADISGyro.turnRobot;
import frc.robot.commands.Drivetrain.MovePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class roundStart extends SequentialCommandGroup {
  /** Creates a new roundStart. */
  public roundStart() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //Need to add: Intake, Shooter, Delay, BallVision, Vision, Hood, Turret, Shuffleboard, and multiple operations
    addCommands(new MovePosition(-4096*10), new turnRobot(180));
  }
}
