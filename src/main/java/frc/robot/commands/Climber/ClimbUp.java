package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class climbUp extends CommandBase {
  /** Creates a new climbUp. */

  double position;
  double posl;
  double posr;
 

  public climbUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    posr = RobotContainer.m_climber.getPositionR();
    posl = RobotContainer.m_climber.getPositionL();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
    {
    
      posr-= 100;
      posl-=100;
      RobotContainer.m_climber.movePositionl(posl);
      RobotContainer.m_climber.movePositionr(posr);
      System.out.println("Posr: " + posr + "Posl " + posl);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

    double endPosr = RobotContainer.m_climber.getPositionR();
    double endPosl =RobotContainer.m_climber.getPositionL();
    RobotContainer.m_climber.movePositionr(endPosr);
    RobotContainer.m_climber.movePositionl(endPosl);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}