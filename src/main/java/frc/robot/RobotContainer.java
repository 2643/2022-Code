// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Shooter;
//import frc.robot.commands.Autonomous.Rountine1;
// import frc.robot.commands.Turret.turretShoot;
// import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Joystick;
// import frc.robot.commands.hoodcm;
// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.Hood;
// import frc.robot.subsystems.Intake;  

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static Shooter m_shooter = new Shooter();




  // public static final BallVision m_ballvision = new BallVision();
  
  
  
  //public static final Hood cm_Hood = new Hood();

  public static Joystick joystick = new Joystick(0);
  //public static final Intake m_intake = new Intake(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    // button1.whenHeld(new moveClimber(Climber.climbDirection.Up));
    // button2.whenHeld(new moveClimber(Climber.climbDirection.Down));
    // turretTest.whenPressed(new turretShoot());
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return (new Rountine1());
  // }
}