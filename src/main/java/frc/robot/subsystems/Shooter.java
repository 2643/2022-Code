// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public static CANSparkMax leftShooter = new CANSparkMax(Constants.LEFT_SHOOTER_PORT, MotorType.kBrushless);
  public static CANSparkMax rightShooter = new CANSparkMax(Constants.RIGHT_SHOOTER_PORT, MotorType.kBrushless);


  double motorP = 0.001;
  double motorI = 0.0000012;
  double motorD = 0.0009;
  // P,I,D,Target,Velocity (RPM),Explanantion
  // 0.0009,0,0,200,142-157,Oscilating
  // 0.0009,0,0.01,200,140-160,Oscilating
  // 0.0009,0,0.02,200,145-154,Oscilating less
  // 0.0009,0,0.04,200,same as before,"Oscilating less, but banging is bad"
  // 0.00045,0,0.04,200,same as before,Oscilating the same
  // 0.001,0,0.04,200,,Oscilating
  // 0.0002,0,0.04,200,,
  // 0.0009,0,0.0009,200,,more oscilation but more banging
  // 0.0009,0,0.0009,200,,
  // 0.0009,0,0.0001,200,,even more oscilation
  // 0.0009,0,0.00001,200,,oscilation
  // 0.0009,0,0.0003,200,,less oscilation
  // 0.0009,0,0.0005,200,,oscilating


  double outputMin = -0.9;
  double outputMax = 0.9;
  int PIDSlot = 0;
  boolean Invertleft = true;
  boolean Invertrightfollow = true;

  //static ShuffleboardTab Tab2022 = Shuffleboard.getTab("2022Tab-1");
  // static NetworkTableEntry targetVelocity = Tab2022.add("Shooter Target Velocity", 0).withSize(2, 2).getEntry();
  // static NetworkTableEntry PValue = Tab2022.add("P Value", 0.001).getEntry();
  // static NetworkTableEntry IValue = Tab2022.add("I Value", 0.0000012).getEntry();
  // static NetworkTableEntry DValue = Tab2022.add("D Value", 0.0009).getEntry();
  //NetworkTableEntry lol4 = Tab2022.a("Max Acceleration", 750).getEntry();
  // static NetworkTableEntry velocityTurret = Tab2022.add("Shooter Velocity(RPM)", 0).withSize(2, 2).getEntry();


  public Shooter() {
    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();
    //leftShooter.setInverted(true);
    leftShooter.getEncoder().setPosition(0);
    leftShooter.getPIDController().setOutputRange(outputMin, outputMax, PIDSlot);
    leftShooter.getPIDController().setP(motorP, PIDSlot);
    leftShooter.getPIDController().setI(motorI, PIDSlot);
    leftShooter.getPIDController().setD(motorD, PIDSlot);

    //leftShooter.getPIDController().setSmartMotionMaxVelocity(500, 0);
    //leftShooter.getEncoder().setVelocityConversionFactor(1);
    rightShooter.follow(leftShooter, true);
  }

  public void setSpeed(double percent)
  {
    leftShooter.getPIDController().setReference(percent, ControlType.kDutyCycle);
  }

  public void setVelSpeed(double speed) {
    leftShooter.getPIDController().setReference(speed, ControlType.kVelocity);
  }

  public double getVelocity(){
    return leftShooter.getEncoder().getVelocity();
  }

  public double getPosition(){
    return leftShooter.getEncoder().getPosition();
  }


  double currentTargetVelocity = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //velocityTurret.setDouble(getVelocity());
    //   System.out.println(leftShooter.getEncoder().getVelocity() + "    " + leftShooter.getEncoder().getVelocityConversionFactor());
    //   if(PValue.getDouble(0) != leftShooter.getPIDController().getP(PIDSlot)) {
    //     leftShooter.getPIDController().setP(PValue.getDouble(0), PIDSlot);
    //   }
    //   if(IValue.getDouble(0) != leftShooter.getPIDController().getI(PIDSlot)) {
    //     leftShooter.getPIDController().setI(IValue.getDouble(0), PIDSlot);
    // }
    //   if(DValue.getDouble(0) != leftShooter.getPIDController().getD(PIDSlot)) {
    //     leftShooter.getPIDController().setD(DValue.getDouble(0), PIDSlot);
    //   }
    // if(targetVelocity.getDouble(1) == 0) {
    //   leftShooter.getPIDController().setReference(0, ControlType.kDutyCycle);
    // } else if(targetVelocity.getDouble(0) != currentTargetVelocity) {
    //   currentTargetVelocity = targetVelocity.getDouble(0);
    //   leftShooter.getPIDController().setReference(targetVelocity.getDouble(0), ControlType.kVelocity, PIDSlot);
    // }
    
  }
}