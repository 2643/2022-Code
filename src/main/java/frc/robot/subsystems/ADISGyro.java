// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.ADIS16470_IMU;


public class ADISGyro extends SubsystemBase {

  /** Creates a new Gyro. */
  ADIS16470_IMU imu = new ADIS16470_IMU();

  public ADISGyro() {}

  public double gyroAngle(){
    return imu.getYComplementaryAngle();
  }

  public void turnClockwiseDegrees(double degrees){
    if(Math.floor(gyroAngle()) == degrees){
      RobotContainer.m_drivetrain.setRightMotorVelocity(0);
      RobotContainer.m_drivetrain.setLeftMotorVelocity(0);
    }
    else if(gyroAngle() > degrees){
      RobotContainer.m_drivetrain.setRightMotorVelocity(-0.5);
      RobotContainer.m_drivetrain.setLeftMotorVelocity(0.5);
    }
    else if(gyroAngle() < degrees){
      RobotContainer.m_drivetrain.setRightMotorVelocity(0.5);
      RobotContainer.m_drivetrain.setLeftMotorVelocity(-0.5);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
