// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.sensors.Pigeon2;


public class PigeonTwo extends SubsystemBase {

  /** Creates a new Gyro. */
  
  Pigeon2 imu = new Pigeon2(Constants.PIGEON_PORT);

  public PigeonTwo() {}

  public double gyroAngle(){
    return -imu.getYaw();
  }

  public void setDegrees(double degrees){
    imu.setYaw(degrees);
  }
  public void turnClockwiseDegrees(double degrees) {
    if(Math.round(gyroAngle()) == Math.round(degrees)) {
      RobotContainer.m_drivetrain.setRightMotorVelocity(0);
      RobotContainer.m_drivetrain.setLeftMotorVelocity(0);
    }
    else if(gyroAngle() > degrees) {
      RobotContainer.m_drivetrain.setRightMotorVelocity(-0.2*Constants.TANKDRIVE_SETPOINT);
      RobotContainer.m_drivetrain.setLeftMotorVelocity(0.2*Constants.TANKDRIVE_SETPOINT);
    }
    else if(gyroAngle() < degrees) {
      RobotContainer.m_drivetrain.setRightMotorVelocity(0.2*Constants.TANKDRIVE_SETPOINT);
      RobotContainer.m_drivetrain.setLeftMotorVelocity(-0.2*Constants.TANKDRIVE_SETPOINT);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //turnClockwiseDegrees(180);
    //System.out.println(gyroAngle());
  }
}
