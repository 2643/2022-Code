package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class moveClimber extends CommandBase {
  /** Creates a new climbUp. */

  private double position;
  private double posl;
  private double posr;
  public static double targetl;
  public static double targetr;
  private double diffErr;

  private Climber.climbDirection m_direction;
 
  public moveClimber(Climber.climbDirection direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climber);
    m_direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    posr = RobotContainer.m_climber.getPositionR();
    posl = RobotContainer.m_climber.getPositionL();

    position = (posl + posr) / 2;
    diffErr = (posl - posr) * Constants.climberGain;

    if (m_direction == Climber.climbDirection.Up) {
      targetr = position + Constants.climberSpeed / 300 + diffErr;
      targetl = position + Constants.climberSpeed - diffErr;
    } else if (m_direction == Climber.climbDirection.Down) {
      targetr = position - Constants.climberSpeed / 300 + diffErr;
      targetl = position - Constants.climberSpeed - diffErr;
    }

    RobotContainer.m_climber.movePositionLimitL(targetl);
    RobotContainer.m_climber.movePositionLimitR(targetr);
    //System.out.println("error L : " + (targetl - posl) + " " + "error R : " + (targetr - posr));
    //System.out.println(posl - posr);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}