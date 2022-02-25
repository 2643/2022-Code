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

  int PIDSlot = 0; 
  double resetPosition = 0;
  double MaxOutput = 0.75;
  double MinOutput = -0.75;
  double ConversionFactor = 2048;
  
  TalonFX rightClimber = new TalonFX(Constants.rightClimberPort);
  TalonFX leftClimber = new TalonFX(Constants.leftClimberPort);
  DigitalInput climberLimitSwitch = new DigitalInput(4);
  
  public static final int timeoutSecondsTalonFX = 1;
  double gearBoxRatio = 1;//gear box ratio is 100:1 change later
  
  double downSoftLimit = 0;
  double upSoftLimit = 1400000;

  double downHardLimit = -9000;
  double upHardLimit = 1409000;

  boolean softLimitEnable = true;

  public static enum climbDirection {
    Up,
    Down
  }

  public Climber() {

    rightClimber.setSelectedSensorPosition(0,0,30);
    rightClimber.set(ControlMode.Position, 0);
    rightClimber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,30);
    rightClimber.config_kP(0,TalonMotorP);
    rightClimber.config_kI(0,TalonMotorI);
    rightClimber.config_kD(0,TalonMotorD);
    rightClimber.config_kF(0, TalonMotorFF);
    //rightClimber.set(ControlMode.MotionMagic,ConversionFactor);
    rightClimber.configPeakOutputForward(1);
    rightClimber.configPeakOutputReverse(-1);

    rightClimber.setInverted(TalonFXInvertType.Clockwise);

    //rightClimber.configMotionAcceleration(250);
    //rightClimber.configMotionCruiseVelocity(500);

    

    //rightClimber.configMotionAcceleration(250);
    //rightClimber.configMotionCruiseVelocity(500);

    
    leftClimber.setSelectedSensorPosition(0,0,30);
    leftClimber.set(ControlMode.Position, 0);
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
  

  public boolean limitswitch(){
    return climberLimitSwitch.get();
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
  
  public void movePositionr(double movePos){
    if((moveClimber.targetr >= upSoftLimit || moveClimber.targetr <= downSoftLimit)){
      if(getPositionR() >= upHardLimit || getPositionR() <= downHardLimit){
        rightClimber.set(ControlMode.Disabled, 0);
      }
    }
    else{
      rightClimber.set(TalonFXControlMode.Position,movePos);
    }
  }

  public void movePositionl(double movepos){
    leftClimber.set(TalonFXControlMode.Position, movepos);
    if((moveClimber.targetl >= upSoftLimit || moveClimber.targetl <= downSoftLimit)){
      if(getPositionR() >= upHardLimit || getPositionR() <= downHardLimit){
        leftClimber.set(ControlMode.Disabled, 0);
      }
    }
    else{
      leftClimber.set(TalonFXControlMode.Position,movepos);
    }
  }

  public void moveResetPositionr(double movePos){
    rightClimber.set(TalonFXControlMode.Position, movePos);
  }

  public void moveResetPositionl(double movepos){
    leftClimber.set(TalonFXControlMode.Position, movepos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //ok lol
    //falcon_motor_dev1.getSelectedSensorPosition();
    System.out.println(rightClimber.getSelectedSensorPosition());
    //rightClimber.set(ControlMode.MotionMagic, 10000);
  }
}
