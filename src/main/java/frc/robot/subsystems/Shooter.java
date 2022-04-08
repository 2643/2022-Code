// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public static CANSparkMax leftShooter = new CANSparkMax(Constants.LEFT_SHOOTER_PORT, MotorType.kBrushless);
  public static CANSparkMax rightShooter = new CANSparkMax(Constants.RIGHT_SHOOTER_PORT, MotorType.kBrushless);


  // double motorP = 0.0018;
  // double motorI = 0.00000;
  // double motorD = 0.00000;
  public static double outputMin = -0.9;
  public static double outputMax = 0.9;
  public static int PIDSlot = 0;
  public static boolean Invertleft = true;
  public static boolean Invertrightfollow = true;

  // ShuffleboardTab Tab2022 = Shuffleboard.getTab("2022Tab-1");
  // NetworkTableEntry maxVelocity = Tab2022.add("Target Velocity", 1).getEntry();
  // NetworkTableEntry PValue = Tab2022.add("P Value", 0.00001).getEntry();
  // NetworkTableEntry IValue = Tab2022.add("I Value", 0).getEntry();
  // NetworkTableEntry DValue = Tab2022.add("D Value", 0).getEntry();
  // NetworkTableEntry lol4 = Tab2022.a("Max Acceleration", 750).getEntry();
  // NetworkTableEntry velocityTurret = Tab2022.add("Current Velocity", 0).getEntry();

  public Shooter() {
    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();
    rightShooter.setInverted(true);
    rightShooter.getEncoder().setPosition(0);
    rightShooter.getPIDController().setOutputRange(outputMin, outputMax, PIDSlot);
    //leftShooter.getPIDController().setSmartMotionMaxVelocity(500, 0);
    //leftShooter.getEncoder().setVelocityConversionFactor(1);
    leftShooter.follow(rightShooter, true);
  }

  public void setSpeed(double percent) {
    rightShooter.getPIDController().setReference(percent, ControlType.kDutyCycle);
  }

  public double getVelocity() {
    return rightShooter.getEncoder().getVelocity();
  }

  public double getPosition() {
    return rightShooter.getEncoder().getPosition();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double velocity = maxVelocity.getDouble(1);
    // double PValue1 = PValue.getDouble(0.0001);
    // double IValue1 = IValue.getDouble(0);
    // double DValue1 = DValue.getDouble(0);
  
    // leftShooter.getPIDController().setP(PValue1, PIDSlot);
    // leftShooter.getPIDController().setI(IValue1, PIDSlot);
    // leftShooter.getPIDController().setD(DValue1, PIDSlot);
    // leftShooter.getPIDController().setReference(velocity, ControlType.kVelocity, PIDSlot);
    
    //velocityTurret.setDouble(getVelocity());
    //System.out.println(leftShooter.getEncoder().getVelocity() + "    " + leftShooter.getEncoder().getVelocityConversionFactor());
  }
}
