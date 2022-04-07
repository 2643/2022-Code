// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.commands.Autonomous.MoveBackwards;
import frc.robot.commands.Autonomous.Routine1;
import frc.robot.commands.Autonomous.Routine2;
import frc.robot.commands.Autonomous.Routine3;
import frc.robot.commands.Autonomous.Routine4;
//import frc.robot.commands.Autonomous.Rountine1;
import frc.robot.commands.Climber.moveClimber;
import frc.robot.commands.Conveyor.*;
import frc.robot.commands.Intake.moveIntake;
import frc.robot.commands.Shooter.shoot;
//import frc.robot.commands.Drivetrain.DifferentialDrive;
// import frc.robot.commands.Turret.turretShoot;
import frc.robot.subsystems.ADISGyro;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ConveyorBelt;
// import frc.robot.subsystems.BallVision;
// import frc.robot.commands.Climber.moveClimber;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
// import frc.robot.subsystems.TurretSubsystem;
// import frc.robot.commands.hoodcm;
// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.Hood;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //SUBSYSTEMS INITIALIZED
  public static final ConveyorBelt conveyorBelt = new ConveyorBelt();
  public static final Drivetrain m_drivetrain = new Drivetrain();
  public static final ADISGyro m_gyro = new ADISGyro();
  public static final Turret m_turret = new Turret();
  public static final Climber m_climber = new Climber();
  public static final Shooter m_shooter = new Shooter();
  public static final Intake m_intake = new Intake(); 

  //JOYSTICK AND OPBOARD INITIALIZED
  public static Joystick driveStick = new Joystick(0);
  public static Joystick opBoard = new Joystick(1);

  //BUTTON INITIALIZATION
  public static JoystickButton driveButton = new JoystickButton(driveStick, 3);

  public static JoystickButton closeShoot = new JoystickButton(opBoard, 1);
  public static JoystickButton autoShoot = new JoystickButton(opBoard, 2);
  public static JoystickButton manualShoot = new JoystickButton(opBoard, 3);
  public static JoystickButton raiseClimber = new JoystickButton(opBoard, 8);
  public static JoystickButton reverseConveyor = new JoystickButton(opBoard, 10); 
  public static JoystickButton forwardConveyor = new JoystickButton(opBoard, 11); 
  public static JoystickButton forwardIntake = new JoystickButton(opBoard, 12); 
  public static JoystickButton lowerClimber = new JoystickButton(opBoard, 15);  

  //public static final Hood cm_Hood = new Hood();


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
    // turretTest.whenPressed(new turretShoot());
    //driveButton.whenHeld(new DifferentialDrive(-0.1, 90, true));
    raiseClimber.whenHeld(new moveClimber(Climber.climbDirection.Up));
    lowerClimber.whenHeld(new moveClimber(Climber.climbDirection.Down));
    forwardIntake.whenHeld(new moveIntake());

    closeShoot.whenHeld(new shoot(Constants.CLOSE_SHOOTER_SPEED).raceWith(new WaitCommand(4)));
    manualShoot.whenHeld(new shoot(Constants.MEDIUM_SHOOTER_SPEED).raceWith(new WaitCommand(4)));
    autoShoot.whenHeld(new shoot(Constants.FAR_SHOOTER_SPEED).raceWith(new WaitCommand(4)));

    forwardConveyor.whileHeld(new conveyorForward());
    reverseConveyor.whileHeld(new conveyorReverse());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     // An ExampleCommand will run in autonomous
     if(Constants.ROUTINE_SELECTOR == 2){
       return (new Routine2(2));
     }
     else if(Constants.ROUTINE_SELECTOR == 3){
      return (new Routine3());
     }
     else if(Constants.ROUTINE_SELECTOR == 4){
       return (new Routine4());
     }
     else{
       return (new Routine1(Constants.AUTONOMOUS_DELAY));
     }
   }
}