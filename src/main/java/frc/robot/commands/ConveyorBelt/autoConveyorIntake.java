// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.ConveyorBelt;

/* MIGHT BE UNECESSARY!!! */
/* MIGHT BE UNECESSARY!!! */
/* MIGHT BE UNECESSARY!!! */

// import java.util.concurrent.DelayQueue;

// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
import frc.robot.RobotContainer;

public class autoConveyorIntake extends CommandBase {
  /** Creates a new autoConveyorIntake. */
  
  private String initState;
  private String finalState;

  public autoConveyorIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_conveyorBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initState = RobotContainer.m_conveyorBelt.getState();
    switch(initState)
    {
      case "empty":
        finalState = "firstonly";
      case "firstonly":
        finalState = "full";
      case "secondonly":
        finalState = "full";
      case "full":
        finalState = "full";
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(initState)
    {
      case "empty":
        // RobotContainer.m_conveyorBelt.setSpeed(Constants.convAutoSpeed);
        if(finalState.equals(RobotContainer.m_conveyorBelt.getState()))
        {
          RobotContainer.m_conveyorBelt.turnOffConv();
          return;
        }
        break;

      case "firstonly":
        // RobotContainer.m_conveyorBelt.setSpeed(Constants.convAutoSpeed);
        if(finalState.equals(RobotContainer.m_conveyorBelt.getState()))
        {
          RobotContainer.m_conveyorBelt.turnOffConv();
          return;
        }
        break;
      case "secondonly":
        // RobotContainer.m_conveyorBelt.setSpeed(-Constants.convAutoSpeed);
        if("firstonly".equals(RobotContainer.m_conveyorBelt.getState()))
        {
          initState = "firstonly";
        }
        break;
      case "full":
        RobotContainer.m_conveyorBelt.turnOffConv();
        return;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_conveyorBelt.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
