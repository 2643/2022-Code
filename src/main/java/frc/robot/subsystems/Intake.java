// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */

  CANSparkMax intakeMotor = new CANSparkMax(Constants.motorPort, MotorType.kBrushed);

  // private static final double kP = 0.000001;
  // private static final double kI = 0.0;
  // private static final double kD = 0;

  // private static final double outputRange = 0;

  public Intake() {

    intakeMotor.setIdleMode(IdleMode.kCoast);

    // intakeMotor.getPIDController().setP(kP);
    // intakeMotor.getPIDController().setI(kI);
    // intakeMotor.getPIDController().setD(kD);
    // intakeMotor.getPIDController().setOutputRange(-1*(outputRange), outputRange);

  }

  public void setSpeed(double speed) {
    
    intakeMotor.getPIDController().setReference(speed, ControlType.kDutyCycle);

  }

  @Override public void periodic () {
    // setSpeed(0.3);
    // This method will be called once per scheduler run
  } 
}
