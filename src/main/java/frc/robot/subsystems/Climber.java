// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.commands.Climber.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Climber extends SubsystemBase {
  /** Creates a new Falcon_500. */
  //TalonFX falcon_motor_dev1 = new TalonFX(3);
  //private static WPI_TalonFX falcon_motor_dev2 = new WPI_TalonFX(3);
  
  //test later
  private static final double TalonMotorP = 0.25;
  private static final double TalonMotorI = 0.0000;
  private static final double TalonMotorD = 0;
  // 0 is overshooting
  private static final double TalonMotorFF = 0.000;
  
  public static final TalonFX rightClimber = new TalonFX(Constants.rightClimberPort);
  public static final TalonFX leftClimber = new TalonFX(Constants.leftClimberPort);
  public static final DigitalInput climberLimitSwitchR = new DigitalInput(4);
  public static final DigitalInput climberLimitSwitchL = new DigitalInput(3);
  
  public static final int timeoutSecondsTalonFX = 1;

  public static enum climbDirection {
    Up,
    Down
  }

  public Climber() {
    rightClimber.setSelectedSensorPosition(0,0,30);
    rightClimber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,30);
    rightClimber.config_kP(0,TalonMotorP);
    rightClimber.config_kI(0,TalonMotorI);
    rightClimber.config_kD(0,TalonMotorD);
    rightClimber.config_kF(0, TalonMotorFF);
    //rightClimber.set(ControlMode.MotionMagic,ConversionFactor);
    rightClimber.configPeakOutputForward(1);
    rightClimber.configPeakOutputReverse(-1);

    //rightClimber.setInverted(TalonFXInvertType.CounterClockwise);

    //rightClimber.configMotionAcceleration(250);
    //rightClimber.configMotionCruiseVelocity(500);

    

    //rightClimber.configMotionAcceleration(250);
    //rightClimber.configMotionCruiseVelocity(500);

    
    leftClimber.setSelectedSensorPosition(0,0,30);
    leftClimber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,30);
    leftClimber.config_kP(0,TalonMotorP);
    leftClimber.config_kI(0,TalonMotorI);
    leftClimber.config_kD(0,TalonMotorD);
    leftClimber.config_kF(0, TalonMotorFF);
    //rightClimber.set(ControlMode.MotionMagic,ConversionFactor);
    leftClimber.configPeakOutputForward(1);
    leftClimber.configPeakOutputReverse(-1);

    leftClimber.setInverted(TalonFXInvertType.Clockwise);
  }
  

  public boolean limitswitchR(){
    return climberLimitSwitchR.get();
  }

  public boolean limitswitchL(){
    return climberLimitSwitchL.get();
  }

  public void setPositionR(double pos){
    rightClimber.setSelectedSensorPosition(pos);
  }

  public void setPositionL(double pos){
    leftClimber.setSelectedSensorPosition(pos);
  }

  public void percentOutputControlResetR(double speed){
    rightClimber.set(ControlMode.PercentOutput, speed);
  }

  public void percentOutputControlResetL(double speed){
    leftClimber.set(ControlMode.PercentOutput, speed);
  }

  public double getPositionR(){
    return rightClimber.getSelectedSensorPosition();
  }

  public double getPositionL(){
    return leftClimber.getSelectedSensorPosition();
  }

  public void movePositionR(double movePos) {
    rightClimber.set(TalonFXControlMode.Position, movePos);
  }

  public void movePositionL(double movePos) {
    leftClimber.set(TalonFXControlMode.Position, movePos);
  }
  
  public void movePositionLimitR(double movePos){
    if (moveClimber.targetr < Constants.upSoftLimit && moveClimber.targetr > Constants.downSoftLimit) {
      rightClimber.set(TalonFXControlMode.Position, movePos);
    } else if (getPositionR() >= Constants.upHardLimit || getPositionR() <= Constants.downHardLimit){
      rightClimber.set(ControlMode.Disabled, 0);
    }
  }

  public void movePositionLimitL(double movePos){
    if (moveClimber.targetl < Constants.upSoftLimit && moveClimber.targetl > Constants.downSoftLimit) {
      leftClimber.set(TalonFXControlMode.Position, movePos);
    } else if (getPositionL() >= Constants.upHardLimit || getPositionL() <= Constants.downHardLimit){
      leftClimber.set(ControlMode.Disabled, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Last resort safety measure for motors by measuring output current
    if (Math.abs(rightClimber.getStatorCurrent()) > 40) {
      rightClimber.set(ControlMode.Disabled, 0);
    }
    if (Math.abs(leftClimber.getStatorCurrent()) > 40) {
      leftClimber.set(ControlMode.Disabled, 0);
    }
  }
}
