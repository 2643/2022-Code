// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.shoot;
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

  public static Joystick opBoard = new Joystick(1);

  //GET ACTUAL PORT NUMBERS
  public static JoystickButton closeShoot = new JoystickButton(opBoard, 10);
  public static JoystickButton manualShoot = new JoystickButton(opBoard, 11);
  public static JoystickButton autoShoot = new JoystickButton(opBoard, 12);


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
    // closeShoot.whenPressed(new shoot(Constants.CLOSE_SHOOTER_SPEED));
    // manualShoot.whenPressed(new shoot(Constants.FAR_SHOOTER_SPEED));
    // autoShoot.whenPressed(new shoot(Constants.FAR_SHOOTER_SPEED));
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