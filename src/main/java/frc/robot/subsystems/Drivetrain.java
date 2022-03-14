// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  TalonFX drivetrainFrontLeftMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_MOTOR);
  TalonFX drivetrainBackLeftMotor = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_MOTOR);

  TalonFX drivetrainFrontRightMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_MOTOR);
  TalonFX drivetrainBackRightMotor = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_MOTOR);

  ADIS16470_IMU imu = new ADIS16470_IMU();

  public static final double TalonFX_F = 0;
  public static final double TalonFX_P = 0.025;
  public static final double TalonFX_I = 0.0000;
  public static final double TalonFX_D = 0.0075;

  public static int loopcounter = 0;
  
  public Drivetrain() {
    TalonFX[] drivetrainMotors = {drivetrainFrontLeftMotor, drivetrainBackLeftMotor,drivetrainFrontRightMotor, drivetrainBackRightMotor};
    // TalonFX[] drivetrainMasterMotors = {drivetrainFrontLeftMotor, drivetrainFrontRightMotor};
    // TalonFX[] drivetrainFollowerMotors = {drivetrainBackLeftMotor, drivetrainBackRightMotor};

    for (TalonFX talon: drivetrainMotors) {
      talon.configFactoryDefault();
      talon.configNominalOutputForward(0);
      talon.configNominalOutputReverse(0);

      talon.configPeakOutputForward(1);
      talon.configPeakOutputReverse(-1);

      talon.configNeutralDeadband(0.001, 50);
      talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);
      talon.setSensorPhase(true);
      talon.setNeutralMode(NeutralMode.Coast);

      talon.config_kP(0, TalonFX_P, 1);
      talon.config_kI(0, TalonFX_I, 1);
      talon.config_kD(0, TalonFX_D, 1);
    }
    drivetrainBackRightMotor.follow(drivetrainFrontRightMotor);
    // drivetrainBackLeftMotor.setInverted(InvertType.FollowMaster);

    // INVERTING RIGHT SIDE SO BOTH SIDES ARE POSITIVE FOR THE SAME DIRECTION
    drivetrainFrontLeftMotor.setInverted(InvertType.InvertMotorOutput);
    drivetrainBackLeftMotor.follow(drivetrainFrontLeftMotor);
    drivetrainBackLeftMotor.setInverted(InvertType.FollowMaster);
    
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

  public void setLeftMotorVelocity(double velocity) {
    drivetrainFrontLeftMotor.set(ControlMode.Velocity, velocity);
    if (loopcounter%50 == 0)
    System.out.println(velocity);
  }

  public void setRightMotorVelocity(double velocity) {
    drivetrainFrontRightMotor.set(ControlMode.Velocity, velocity);
  }

  public void setMotorVelocity(double velocity) {
    setLeftMotorVelocity(velocity);
    setRightMotorVelocity(velocity);
  }

  public void setLeftMotorPercentOutput(double percent) {
    drivetrainFrontLeftMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setRightMotorPercentOutput(double percent) {
    drivetrainFrontRightMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setMotorPercentOutput(double percent){
    setLeftMotorPercentOutput(percent);
    setRightMotorPercentOutput(percent);
    }

  public void setLeftMotorPosition(double position) {
    drivetrainFrontLeftMotor.set(ControlMode.MotionMagic, position);
    drivetrainFrontLeftMotor.configMotionCruiseVelocity(Constants.DRIVETRAIN_VELOCITY);
    drivetrainFrontLeftMotor.configMotionAcceleration(Constants.DRIVETRAIN_ACCELERATION);
  }

  public void setRightMotorPosition(double position) {
    drivetrainFrontRightMotor.set(ControlMode.MotionMagic, position);
    drivetrainFrontRightMotor.configMotionCruiseVelocity(Constants.DRIVETRAIN_VELOCITY);
    drivetrainFrontRightMotor.configMotionAcceleration(Constants.DRIVETRAIN_ACCELERATION);
  }

  public void setMotorPosition(double position) {
    setLeftMotorPosition(position);
    setRightMotorPosition(position);
  }

  public double getLeftMotorPosition() {
    return drivetrainFrontLeftMotor.getSelectedSensorPosition();
  }

  public double getRightMotorPosition() {
    return drivetrainFrontRightMotor.getSelectedSensorPosition();
  }

  public void resetMotorEncoders() {
    drivetrainFrontLeftMotor.setSelectedSensorPosition(0);
    drivetrainFrontRightMotor.setSelectedSensorPosition(0);
  }

  public double gyroAngle(){
    return imu.getAngle();
  }

  public void turnClockwiseDegrees(double degrees){
    if(Math.round(gyroAngle()) == degrees){
      setRightMotorVelocity(0);
      setLeftMotorVelocity(0);
    }
    else if(gyroAngle() > degrees){
      setRightMotorVelocity(-0.5);
      setLeftMotorVelocity(0.5);
    }
    else if(gyroAngle() < degrees){
      setRightMotorVelocity(0.5);
      setLeftMotorVelocity(-0.5);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (++loopcounter%50 == 0) {
      // System.out.println("Actual Output: " + drivetrainBackLeftMotor.getSelectedSensorVelocity() + " " + drivetrainBackLeftMotor.getStatorCurrent());
      loopcounter = 0;
    }
  }
}