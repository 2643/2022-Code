// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Climber extends SubsystemBase {
  TalonFX rightClimber = new TalonFX(Constants.rightClimberPort);
  TalonFX leftClimber = new TalonFX(Constants.leftClimberPort);

  AnalogPotentiometer pot = new AnalogPotentiometer(Constants.potentiometerChannel, 180, 30);


  public Climber() {
    leftClimber.set(ControlMode.Follower,Constants.rightClimberPort);
  }





  public void Speed(double speed){
    rightClimber.set(ControlMode.PercentOutput,speed);
  }

  


  @Override
  public void periodic() {
    double position = pot.get();
    System.out.println(position);
  }
}
