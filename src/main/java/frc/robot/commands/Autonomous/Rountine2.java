// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ADISGyro.turnRobot;
import frc.robot.commands.Drivetrain.MovePosition;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Rountine2 extends SequentialCommandGroup {
  /** Creates a new Rountine2. */
  private final Timer m_timer = new Timer();
  public Rountine2(double delay) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_timer.reset();
    m_timer.start();
    if(m_timer.get()==delay){
      addCommands(new MovePosition(-4096*10), new turretShoot(), new hood(), new motorSpeed(), new turnRobot(180), new MovePosition(4096*10), new Intake(), new turnRobot(180), new turretShoot(), new hood(), new motorSpeed());   
     }
    m_timer.stop();
  }
}
