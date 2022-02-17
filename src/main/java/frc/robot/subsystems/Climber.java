// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {

  // need to test PID and FF values
  private final double CANSparkMotorP = 0.02;
  private final double CANSparkMotorI = 0.0000;
  private final double CANSparkMotorD = 0;
  private final double CANSparkMotorFF = 0.0001;

  int PIDSlot = 0; 
  double resetPosition = 0;
  double MaxOutput = 0.75;
  double MinOutput = -0.75;
  double ConversionFactor = 4096;
  
  CANSparkMax rightClimber = new CANSparkMax(Constants.rightClimberPort,MotorType.kBrushless);
  CANSparkMax leftClimber = new CANSparkMax(Constants.leftClimberPort, MotorType.kBrushless);
  DigitalInput climberLimitSwitch = new DigitalInput(0);//need to figure out port number later

  

  



  double gearBoxRatio = 0.0; // gear ratio
  public Object m_climber;

  public Climber() {

    rightClimber.getPIDController().setP(CANSparkMotorP, 0);
    rightClimber.getPIDController().setI(CANSparkMotorI, 0);
    rightClimber.getPIDController().setD(CANSparkMotorD, 0);
    rightClimber.getPIDController().setFF(CANSparkMotorFF, 0);
    rightClimber.getPIDController().setOutputRange(MinOutput, MaxOutput);
    rightClimber.getEncoder().setPosition(resetPosition);
    rightClimber.getEncoder().setPositionConversionFactor(ConversionFactor);
    rightClimber.setIdleMode(IdleMode.kCoast);

    rightClimber.getEncoder().setPositionConversionFactor(ConversionFactor);
    leftClimber.follow(rightClimber);

  }

  public boolean limitswitch(){
    
    return climberLimitSwitch.get();
    
  }
  public void setPosition(double pos){
    rightClimber.getEncoder().setPosition(pos);
  }
  public void movePosition(double movePos)
  {
    rightClimber.getPIDController().setReference(movePos, ControlType.kSmartMotion);
  }
 
  

  @Override
  public void periodic() {
  }

public void rightClimber(double speed) {
}
}
