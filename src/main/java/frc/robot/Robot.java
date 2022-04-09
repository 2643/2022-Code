// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Climber.resetPosition;
// import frc.robot.commands.FindBall;
import frc.robot.commands.Drivetrain.Tankdrive;
import frc.robot.commands.Intake.releaseLatches;
import frc.robot.commands.Turret.driverControl;
import frc.robot.subsystems.ConveyorBelt;

// import frc.robot.commands.hoodcm;
// import frc.robot.subsystems.Hood;
// import frc.robot.subsystems.Intake;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
//   private boolean resetTurretDone = false;
  private boolean releaseIntakeDone = false;
  private boolean resetClimberDone = false;
  
//   public static boolean canDriverControl = true;
NetworkTableEntry ShuffleBoardDelay = Shuffleboard.getTab("2022Robot").add("Autonomous Delay", 0).withSize(2, 2).getEntry();
NetworkTableEntry ballAtTopLimitSwitch = Shuffleboard.getTab("2022Robot").getLayout("Conveyor(Green = Ball and Red = No Ball)", BuiltInLayouts.kGrid).withSize(3, 2).add("Ball at Top", false).withWidget(BuiltInWidgets.kBooleanBox).withSize(3, 3).withPosition(1, 1).getEntry();
NetworkTableEntry ballAtBottomLimitSwitch = Shuffleboard.getTab("2022Robot").getLayout("Conveyor(Green = Ball and Red = No Ball)", BuiltInLayouts.kGrid).withSize(3, 2).add("Ball at Bottom", false).withWidget(BuiltInWidgets.kBooleanBox).withSize(3, 3).withPosition(1, 2).getEntry();


  private RobotContainer m_robotContainer;
  //public hoodcm m_hoodccm = new hoodcm(); 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    RobotContainer.m_conveyorBelt.setSpeed(0);  
    m_robotContainer = new RobotContainer(); 
    //CommandScheduler.getInstance().setDefaultCommand(RobotContainer.cm_Hood, new hoodcm());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    //CommandScheduler.getInstance().setDefaultCommand(RobotContainer.m_turret, new SequentialCommandGroup(new resetPosition(), new driverControl()));
    //NetworkTableEntry lol = Shuffleboard.getTab("2022Robot").add("Hi", 1).
    Constants.AUTONOMOUS_DELAY = ShuffleBoardDelay.getDouble(0);
    if(!resetClimberDone) {
      CommandScheduler.getInstance().schedule(new resetPosition());
      resetClimberDone = true;
    }
    if (!releaseIntakeDone) {
      CommandScheduler.getInstance().schedule(new releaseLatches(Constants.INTAKE_SERVO_LATCH_DEGREES));
      releaseIntakeDone = true;
    }
    RobotContainer.m_conveyorBelt.setSpeed(0);  

    // else {
    //   CommandScheduler.getInstance().schedule(true, new driverControl());
    // }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      System.out.println("Calling autonomus command");
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Constants.slowMode = true;

    if(!resetClimberDone) {
      CommandScheduler.getInstance().schedule(new resetPosition());
      resetClimberDone = true;
    }
    if (!releaseIntakeDone) {
      CommandScheduler.getInstance().schedule(new releaseLatches(Constants.INTAKE_SERVO_LATCH_DEGREES));
      releaseIntakeDone = true;
    }
    RobotContainer.m_conveyorBelt.setSpeed(0);  
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // CommandScheduler.getInstance().schedule(new FindBall());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // CommandScheduler.getInstance().schedule(new FindBall());
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.m_drivetrain, new Tankdrive());
    ballAtTopLimitSwitch.setBoolean(ConveyorBelt.conviRSens2.get());
    ballAtBottomLimitSwitch.setBoolean(ConveyorBelt.conviRSens1.get());
    CommandScheduler.getInstance().schedule(true, new driverControl());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
