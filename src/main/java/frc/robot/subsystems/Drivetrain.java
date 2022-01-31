// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  TalonFX drivetrainFrontLeftMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_MOTOR);
  TalonFX drivetrainBackLeftMotor = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_MOTOR);

  TalonFX drivetrainFrontRightMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_MOTOR);
  TalonFX drivetrainBackRightMotor = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_MOTOR);

  
  public Drivetrain() {
    TalonFX[] drivetrainMotors = {drivetrainFrontLeftMotor, drivetrainBackLeftMotor,drivetrainFrontRightMotor, drivetrainBackRightMotor};
    // TalonFX[] drivetrainMasterMotors = {drivetrainFrontLeftMotor, drivetrainFrontRightMotor};
    // TalonFX[] drivetrainFollowerMotors = {drivetrainBackLeftMotor, drivetrainBackRightMotor};

    for (TalonFX talon: drivetrainMotors) {
      talon.configFactoryDefault();
      talon.configNominalOutputForward(0);
      talon.configNominalOutputReverse(0);

      talon.configPeakOutputForward(0.75);
      talon.configPeakOutputReverse(-0.75);

      talon.configNeutralDeadband(0.01, 50);
      talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);
      talon.setSensorPhase(true);
      talon.setNeutralMode(NeutralMode.Coast);
    }
    drivetrainBackLeftMotor.follow(drivetrainFrontLeftMotor);
    drivetrainBackLeftMotor.setInverted(InvertType.FollowMaster);

    // INVERTING RIGHT SIDE SO BOTH SIDES ARE POSITIVE FOR THE SAME DIRECTION
    drivetrainFrontRightMotor.setInverted(InvertType.InvertMotorOutput);
    drivetrainBackRightMotor.follow(drivetrainFrontRightMotor);
    drivetrainBackRightMotor.setInverted(InvertType.FollowMaster);
    
  }

  public void setLeftMotorSpeed(double speed) {
    drivetrainFrontLeftMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setRightMotorSpeed(double speed) {
    drivetrainFrontRightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setMotorSpeed(double speed) {
    setLeftMotorSpeed(speed);
    setRightMotorSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
