// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    // Creates a new Drivetrain. 
   TalonFX drivetrainFrontLeftMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_MOTOR);
   TalonFX drivetrainBackLeftMotor = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_MOTOR);

   TalonFX drivetrainFrontRightMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_MOTOR);
   TalonFX drivetrainBackRightMotor = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_MOTOR);

   public static final double TalonFX_F = 0;
   public static final double TalonFX_P = 0.025;
   public static final double TalonFX_I = 0.0000;
   public static final double TalonFX_D = 0.0075;
  
   public Drivetrain() {
      TalonFX[] drivetrainMotors = {drivetrainFrontLeftMotor, drivetrainBackLeftMotor,drivetrainFrontRightMotor, drivetrainBackRightMotor};
      TalonFX[] drivetrainMasterMotors = {drivetrainFrontLeftMotor, drivetrainFrontRightMotor};
      TalonFX[] drivetrainFollowerMotors = {drivetrainBackLeftMotor, drivetrainBackRightMotor};

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
     drivetrainBackLeftMotor.setInverted(InvertType.FollowMaster);

  //   // INVERTING RIGHT SIDE SO BOTH SIDES ARE POSITIVE FOR THE SAME DIRECTION
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
     System.out.println(velocity);
   }

   public void setRightMotorVelocity(double velocity) {
     drivetrainFrontRightMotor.set(ControlMode.Velocity, velocity);
   }

   public void setMotorVelocity(double velocity) {
     setLeftMotorVelocity(velocity);
     setRightMotorVelocity(velocity);
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(drivetrainBackLeftMotor.getSelectedSensorVelocity() + " " + drivetrainBackLeftMotor.getStatorCurrent());
  }
}
*/