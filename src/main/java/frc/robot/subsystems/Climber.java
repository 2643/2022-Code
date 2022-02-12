// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;




public class Climber extends SubsystemBase {

  private final double TalonMotorP = 0.02;
  private final double TalonkMotorI = 0.00001;
  private final double TalonMotorD = 0;
  
  public static final int timeoutSecondsTalonFX = 1;
  
  TalonFX rightClimber = new TalonFX(Constants.rightClimberPort);
  TalonFX leftClimber = new TalonFX(Constants.leftClimberPort);

  AnalogPotentiometer pot = new AnalogPotentiometer(Constants.potentiometerChannel, 180, 30);
  double gearBoxRatio = 0.0/*gear ratio*/;
  public Climber() {
    rightClimber.configFactoryDefault();
    rightClimber.setSelectedSensorPosition(0,0,30);
    rightClimber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,30);
    rightClimber.config_kP(0,TalonMotorP);
    rightClimber.config_kI(0,TalonkMotorI);
    rightClimber.config_kD(0,TalonMotorD);
    rightClimber.set(ControlMode.MotionMagic,2048);
    rightClimber.configPeakOutputForward(0.5);
    rightClimber.configPeakOutputReverse(-0.5);

    rightClimber.configMotionAcceleration(500);
    rightClimber.configMotionCruiseVelocity(1000);

    rightClimber.configNeutralDeadband(0.001);
    leftClimber.set(ControlMode.Follower,Constants.rightClimberPort);
  }

  public void speedClimb(double speed){
    rightClimber.set(ControlMode.PercentOutput,speed);
  }
  public double potPosition()
  {
    return pot.get()*gearBoxRatio;
  }

  


  @Override
  public void periodic() {
    potPosition();
  }
}
