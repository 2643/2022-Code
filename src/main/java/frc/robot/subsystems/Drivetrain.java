// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  TalonFX drivetrainFrontLeftMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_MOTOR);
  TalonFX drivetrainBackLeftMotor = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_MOTOR);

  TalonFX drivetrainFrontRightMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_MOTOR);
  TalonFX drivetrainBackRightMotor = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_MOTOR);

  // WPI_TalonFX WPI_drivetrainFrontLeftMotor = new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_MOTOR);
  // WPI_TalonFX WPI_drivetrainBackLeftMotor = new WPI_TalonFX(Constants.DRIVETRAIN_BACK_LEFT_MOTOR);

  // WPI_TalonFX WPI_drivetrainFrontRightMotor = new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_MOTOR);
  // WPI_TalonFX WPI_drivetrainBackRightMotor = new WPI_TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_MOTOR);

  // MotorControllerGroup m_left = new MotorControllerGroup((MotorController)WPI_drivetrainFrontLeftMotor);
  // MotorControllerGroup m_right = new MotorControllerGroup((MotorController)WPI_drivetrainFrontRightMotor);

  // public DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);


  public static final double TalonFX_F = 0;
  public static final double TalonFX_P = 0.025;
  public static final double TalonFX_I = 0.0000;
  public static final double TalonFX_D = 0.0075;

  public static final double Position_TalonFX_P = 0.025;
  public static final double Position_TalonFX_I = 0.0000;
  public static final double Position_TalonFX_D = 0;

  public static int loopcounter = 0;
  
  public Drivetrain() {

    resetMotorEncoders();

    /*WPI_drivetrainFrontLeftMotor, WPI_drivetrainBackLeftMotor, WPI_drivetrainFrontRightMotor, WPI_drivetrainBackRightMotor*/
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

      talon.config_kP(1, TalonFX_P, 1);
      talon.config_kI(1, TalonFX_I, 1);
      talon.config_kD(1, TalonFX_D, 1);
    }

    drivetrainBackRightMotor.follow(drivetrainFrontRightMotor);
    // drivetrainBackLeftMotor.setInverted(InvertType.FollowMaster);

    // INVERTING RIGHT SIDE SO BOTH SIDES ARE POSITIVE FOR THE SAME DIRECTION
    drivetrainFrontLeftMotor.setInverted(InvertType.InvertMotorOutput);
    drivetrainBackLeftMotor.follow(drivetrainFrontLeftMotor);
    drivetrainBackLeftMotor.setInverted(InvertType.FollowMaster);

    //WPI_drivetrainBackRightMotor.follow(WPI_drivetrainFrontRightMotor);
    // drivetrainBackLeftMotor.setInverted(InvertType.FollowMaster);

    // INVERTING RIGHT SIDE SO BOTH SIDES ARE POSITIVE FOR THE SAME DIRECTION
    // WPI_drivetrainFrontLeftMotor.setInverted(InvertType.InvertMotorOutput);
    // WPI_drivetrainBackLeftMotor.follow(WPI_drivetrainFrontLeftMotor);
    // WPI_drivetrainBackLeftMotor.setInverted(InvertType.FollowMaster);
    // WPI_drivetrainFrontRightMotor.setNeutralMode(NeutralMode.Brake);
    // WPI_drivetrainFrontLeftMotor.setNeutralMode(NeutralMode.Coast);
    // WPI_drivetrainFrontLeftMotor.configMotionAcceleration(5);
    // WPI_drivetrainFrontRightMotor.configMotionAcceleration(5);
    // m_drive.setMaxOutput(0.1);
    
    //WPI_drivetrainFrontLeftMotor.configMotionCruiseVelocity(5);
    //WPI_drivetrainFrontRightMotor.configMotionCruiseVelocity(5);
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
    drivetrainFrontRightMotor.selectProfileSlot(0, 0);
    drivetrainFrontLeftMotor.set(ControlMode.Velocity, velocity);
  }

  public void setRightMotorVelocity(double velocity) {
    drivetrainFrontRightMotor.selectProfileSlot(0, 0);
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
    drivetrainFrontLeftMotor.selectProfileSlot(1, 0);
    drivetrainFrontLeftMotor.set(ControlMode.MotionMagic, position);
    drivetrainFrontLeftMotor.configMotionCruiseVelocity(Constants.DRIVETRAIN_VELOCITY);
    drivetrainFrontLeftMotor.configMotionAcceleration(Constants.DRIVETRAIN_ACCELERATION);
  }

  public void setRightMotorPosition(double position) {
    drivetrainFrontRightMotor.selectProfileSlot(1, 0);
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

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (++loopcounter%50 == 0) {
    //   System.out.println("Actual Output: " + drivetrainBackLeftMotor.getSelectedSensorVelocity() + " " + drivetrainBackLeftMotor.getStatorCurrent());
    //   loopcounter = 0;
    // }
    //setMotorPercentOutput(0.3);
  }
}
