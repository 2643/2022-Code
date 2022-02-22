// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Climber extends SubsystemBase {
  /** Creates a new Falcon_500. */
  //TalonFX falcon_motor_dev1 = new TalonFX(3);
  //private static WPI_TalonFX falcon_motor_dev2 = new WPI_TalonFX(3);
  


  private final double TalonMotorP = 0.1;
  private final double TalonMotorI = 0.0000;
  private final double TalonMotorD = 0.2;
  // 0 is overshooting
  private final double TalonMotorFF = 0.000;

  int PIDSlot = 0; 
  double resetPosition = 0;
  double MaxOutput = 0.75;
  double MinOutput = -0.75;
  double ConversionFactor = 2048;
  
  TalonFX rightClimber = new TalonFX(Constants.rightClimberPort);
  //TalonFX leftClimber = new TalonFX(Constants.leftClimberPort);
  DigitalInput climberLimitSwitch = new DigitalInput(4);
  
  public static final int timeoutSecondsTalonFX = 1;
  double gearBoxRatio = 1;
  
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

    //rightClimber.configMotionAcceleration(250);
    //rightClimber.configMotionCruiseVelocity(500);

    rightClimber.configNeutralDeadband(0.001);
    //leftClimber.set(ControlMode.Follower,Constants.rightClimberPort);
  }
  public boolean limitswitch(){
    
    return climberLimitSwitch.get();
    
  }
  public void setPosition(double pos){
    rightClimber.setSelectedSensorPosition(pos);
  }
  public void velocitycontrol(double speed){
    rightClimber.set(TalonFXControlMode.Velocity,speed);

  }
  public void movePosition(double movePos)
  {
    rightClimber.set(TalonFXControlMode.Position,movePos);
  }
 
  public double getPosition(){
    return rightClimber.getSelectedSensorPosition();
  }

    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //ok lol
    //falcon_motor_dev1.getSelectedSensorPosition();
    //System.out.println(rightClimber.getSelectedSensorPosition());
    //rightClimber.set(ControlMode.MotionMagic, 10000);
  }

  

}
