// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.moveIntake;
import frc.robot.commands.Autonomous.Rountine3;
//import frc.robot.commands.Autonomous.Rountine1;
import frc.robot.commands.Climber.moveClimber;
import frc.robot.commands.Drivetrain.DifferentialDrive;
// import frc.robot.commands.Turret.turretShoot;
import frc.robot.subsystems.ADISGyro;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.BallVision;
// import frc.robot.commands.Climber.moveClimber;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Joystick;
// import frc.robot.commands.hoodcm;
// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.Hood;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static final Drivetrain m_drivetrain = new Drivetrain();
  public static final ADISGyro m_gyro = new ADISGyro();
  // public static TurretSubsystem m_turret = new TurretSubsystem();
  public static final Climber m_climber = new Climber();

  public static Joystick driveStick = new Joystick(0);
  public static Joystick opBoard = new Joystick(1);

  public static JoystickButton driveButton = new JoystickButton(driveStick, 3);
  public static JoystickButton raiseClimber = new JoystickButton(opBoard, 8);
  public static JoystickButton lowerClimber = new JoystickButton(opBoard, 15);

  // public static final BallVision m_ballvision = new BallVision();
  
  //public static final Hood cm_Hood = new Hood();

  public static Joystick joystick = new Joystick(0);
  public static final Intake m_intake = new Intake(); 
  public static Joystick opboard = new Joystick(1);
  public static JoystickButton button12 = new JoystickButton(opboard, 12); 

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
    driveButton.whenHeld(new DifferentialDrive(-0.1, 90, true));
    raiseClimber.whenHeld(new moveClimber(Climber.climbDirection.Up));
    lowerClimber.whenHeld(new moveClimber(Climber.climbDirection.Down));
    button12.whenHeld(new moveIntake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     // An ExampleCommand will run in autonomous
     return (new Rountine3());
   }
}