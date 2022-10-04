// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.AutoShoot.autoShoot;
//import frc.robot.commands.Autonomous.MoveBackwards;
import frc.robot.commands.Autonomous.Routine1;
import frc.robot.commands.Autonomous.Routine2;
import frc.robot.commands.Autonomous.Routine3;
import frc.robot.commands.Autonomous.Routine4;
import frc.robot.commands.Autonomous.Routine5;
// import frc.robot.commands.Autonomous.Routine4;
//import frc.robot.commands.Autonomous.Rountine1;
import frc.robot.commands.Climber.moveClimber;
import frc.robot.commands.Climber.rightUp;
import frc.robot.commands.ConveyorBelt.*;
import frc.robot.commands.Intake.moveIntake;
import frc.robot.commands.Intake.moveIntakeReverse;
//import frc.robot.commands.Shooter.shoot;
//import frc.robot.commands.Drivetrain.DifferentialDrive;
// import frc.robot.commands.Turret.turretShoot;
import frc.robot.subsystems.PigeonTwo;
import frc.robot.subsystems.BallVision;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ConveyorBelt;
// import frc.robot.subsystems.BallVision;
// import frc.robot.commands.Climber.moveClimber;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
// import frc.robot.subsystems.TurretSubsystem;
// import frc.robot.commands.hoodcm;
// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.Hood;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.Robot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SendableChooser<Command> m_chooser = new SendableChooser<>();
  ComplexWidget ShuffleBoardAutonomousRoutines = Shuffleboard.getTab("2022Robot").add("Autonoumous Routines Selector", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 2);
  public final Command m_Routine1 = new Routine1(Constants.AUTONOMOUS_DELAY);
  public final Command m_Routine2 = new Routine2(Constants.AUTONOMOUS_DELAY);
  public final Command m_Routine3 = new Routine3(Constants.AUTONOMOUS_DELAY);
  public final Command m_Routine4 = new Routine4(Constants.AUTONOMOUS_DELAY);
  public final Command m_Routine5 = new Routine5();

  
  //SUBSYSTEMS INITIALIZED
  public static final Drivetrain m_drivetrain = new Drivetrain();
  public static final Climber m_climber = new Climber();
  public static final Turret m_turret = new Turret();
  public static final Hood m_hood = new Hood();
  public static final ConveyorBelt m_conveyorBelt = new ConveyorBelt();
  public static final Intake m_intake = new Intake(); 
  public static final Shooter m_shooter = new Shooter();
  public static final PigeonTwo m_gyro = new PigeonTwo();
  public static final BallVision m_ballvision = new BallVision(); 

  //JOYSTICK AND OPBOARD INITIALIZED
  public static Joystick joyStick = new Joystick(0);
  public static Joystick opBoard = new Joystick(1);

  //BUTTON INITIALIZATION
  public static JoystickButton driveButton = new JoystickButton(joyStick, 3);
  public static JoystickButton shootButton = new JoystickButton(joyStick, 1);

  public static JoystickButton closeShoot = new JoystickButton(opBoard, 1);
  public static JoystickButton mediumShoot = new JoystickButton(opBoard, 2);
  public static JoystickButton farShoot = new JoystickButton(opBoard, 3);
  //public static JoystickButton superMediumShoot = new JoystickButton(opBoard, 4);
  //public static JoystickButton superSuperCloseShoot = new JoystickButton(opBoard, 5);
  //public static JoystickButton superCloseShoot = new JoystickButton(opBoard, 6);
  public static JoystickButton autoIntake = new JoystickButton(opBoard, 7);
  public static JoystickButton raiseClimber = new JoystickButton(opBoard, 8);
  public static JoystickButton reverseIntake = new JoystickButton(opBoard, 9);  
  public static JoystickButton reverseConveyor = new JoystickButton(opBoard, 10); 
  //public static JoystickButton forwardConveyor = new JoystickButton(opBoard, 11); 
  public static JoystickButton forwardConveyor = new JoystickButton(joyStick, 2); 

  public static JoystickButton forwardIntake = new JoystickButton(opBoard, 12);
  public static JoystickButton rightUp = new JoystickButton(opBoard, 13);
  public static JoystickButton lowerClimber = new JoystickButton(opBoard, 15);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_chooser.setDefaultOption("MOVE FORWARD, GET SECOND BALL, TURN, AND SHOOT", m_Routine2);
    m_chooser.addOption("MOVE BACK AND SHOOT", m_Routine1);
    m_chooser.addOption("MOVE FORWARD, GET SECOND BALL, TURN, AND SHOOT", m_Routine2);
    m_chooser.addOption("MOVE BACK", m_Routine3);
    m_chooser.addOption("Needs to be tested", m_Routine4);
    m_chooser.addOption("Turn 180", m_Routine5);
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
    // driveButton.whenHeld(new DifferentialDrive(-0.1, 90, true));
    raiseClimber.whenHeld(new moveClimber(Climber.climbDirection.Up));
    lowerClimber.whenHeld(new moveClimber(Climber.climbDirection.Down));
    rightUp.whenHeld(new rightUp());
    forwardIntake.whenHeld(new moveIntake());
    reverseIntake.whenHeld(new moveIntakeReverse());
    autoIntake.whenHeld(new ParallelCommandGroup(new moveIntake(), new conveyorForward()));

    // closeShoot.whenHeld(new shoot(Constants.CLOSE_SHOOTER_VEL).raceWith(new WaitCommand(4)));
    // mediumShoot.whenHeld(new shoot(Constants.MEDIUM_SHOOTER_VEL).raceWith(new WaitCommand(4)));
    // farShoot.whenHeld(new shoot(Constants.FAR_SHOOTER_VEL).raceWith(new WaitCommand(4)));
    // superCloseShoot.whenHeld(new shoot(Constants.SUPER_CLOSE_SHOOTER_SPEED));
    // closeShoot.whenHeld(new shoot(Constants.CLOSE_SHOOTER_SPEED));
    // manualShoot.whenHeld(new shoot(Constants.MEDIUM_SHOOTER_SPEED));
    // autoShoot.whenHeld(new shoot(Constants.FAR_SHOOTER_SPEED));
    // superCloseShoot.whenHeld(new shoot(Constants.SUPER_CLOSE_SHOOTER_SPEED));
    // superSuperCloseShoot.whenHeld(new shoot(Constants.SUPER_SUPER_CLOSE_SHOOTER_SPEED));
    // superMediumShoot.whenHeld(new shoot(Constants.SUPER_MEDIUM_SHOOTER_SPEED));

    forwardConveyor.whileHeld(new conveyorForward());
    reverseConveyor.whileHeld(new conveyorReverse());
    //shootButton.whenHeld(new autoShoot(Constants.DISTANCE.getDouble(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     // An ExampleCommand will run in autonomous
     return m_chooser.getSelected();
     //return (new Routine2(0));
   }
}