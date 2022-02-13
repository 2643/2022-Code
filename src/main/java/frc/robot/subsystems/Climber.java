// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class Climber extends SubsystemBase {

  // need to test PID values
  private final double CANSparkMotorP = 0.02;
  private final double CANSparkMotorI = 0.00001;
  private final double CANSparkMotorD = 0;

  int PIDSlot = 0; 
  double resetPosition = 0;
  double MaxOutput = 0.75;
  double MinOutput = -0.75;
  double ConversionFactor = 4096;
  
  CANSparkMax rightClimber = new CANSparkMax(Constants.rightClimberPort,MotorType.kBrushless);
  CANSparkMax leftClimber = new CANSparkMax(Constants.leftClimberPort, MotorType.kBrushless);

  AnalogPotentiometer pot = new AnalogPotentiometer(Constants.potentiometerChannel, 180, 30);
  double gearBoxRatio = 0.0; // gear ratio

  public Climber() {

    rightClimber.getPIDController().setP(CANSparkMotorP);
    rightClimber.getPIDController().setI(CANSparkMotorI);
    rightClimber.getPIDController().setD(CANSparkMotorD);
    rightClimber.getPIDController().setOutputRange(MinOutput, MaxOutput);
    rightClimber.getEncoder().setPosition(resetPosition);
    rightClimber.getEncoder().setPositionConversionFactor(ConversionFactor);
    rightClimber.setIdleMode(IdleMode.kCoast);

    rightClimber.getEncoder().setPositionConversionFactor(ConversionFactor);
    leftClimber.follow(rightClimber);

  }

  public void speedClimb(double speed){
    rightClimber.getPIDController().setReference(speed, ControlType.kSmartVelocity, PIDSlot);
  }

  public double potPosition() {
    return pot.get() * gearBoxRatio;
  }

  @Override
  public void periodic() {
    potPosition();
  }
}
