// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Hood extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(Constants.HOOD_MOTOR_PORT, MotorType.kBrushless);
  //private DigitalInput maxPositionTrue = new DigitalInput(Constants.maxHoodLimitPort);

  //sets PID variables
    private static final double kP = 0.000001;
    private static final double kI = 0.0;
    private static final double kD = 0;

    private static final double outputRange = 0.5;
    private static final int slotID = 0;

    private static final double smartMotionMaxVel = 0;
    private static final double smartMotionMaxAccel = 0;

    private static final double posConversionFactor = 4096;

  //max/min position that the hood can move to
  //private static final double maxPosition = 10000;
  private static final double minPosition = 0;
  ShuffleboardTab HoodTab = Shuffleboard.getTab("HoodTab");
  NetworkTableEntry positionHood = HoodTab.add("Position Value", 0).getEntry();
  NetworkTableEntry proportionalValue = HoodTab.add("P Value", 0.00005).getEntry();
  NetworkTableEntry integralValue = HoodTab.add("I Value", 0.000000).getEntry();
  NetworkTableEntry derivativeValue = HoodTab.add("D Value", 0.00019).getEntry();
  NetworkTableEntry maxVel = HoodTab.add("Max Velocity", 2000).getEntry();
  NetworkTableEntry maxAcc = HoodTab.add("Max Acceleration", 750).getEntry();
  NetworkTableEntry upLimit = HoodTab.add("Up Limit", 0).getEntry();


  /** Creates a new Hood. */
public Hood() {
    //sets PID values
    motor.restoreFactoryDefaults();
    motor.getEncoder().setPositionConversionFactor(posConversionFactor);
    motor.getEncoder().setPosition(0);

    motor.getPIDController().setP(kP, slotID);
    motor.getPIDController().setI(kI, slotID);
    motor.getPIDController().setD(kD, slotID);

    motor.getPIDController().setOutputRange(-1*(outputRange), outputRange);
    motor.getPIDController().setSmartMotionMaxVelocity(smartMotionMaxVel, slotID);
    motor.getPIDController().setSmartMotionMaxAccel(smartMotionMaxAccel, slotID);

}
  //resets encoder to 0
public void setEncoder(){
    motor.getEncoder().setPosition(0);
}
  //stops the motion of the hood/robot is at a resting position
public void stopMove(){
    motor.getPIDController().setReference(0, ControlType.kDutyCycle);
}

//   public boolean atTopPos(){
//     return maxPositionTrue.get();
//   }


public boolean atBottomPos(){
    return (motor.getEncoder().getPosition() == minPosition);
}

public void hoodPID(double position){
    if(getPosition() >= Constants.HOOD_DOWN_HARD_LIMIT && getPosition() <= Constants.HOOD_UP_HARD_LIMIT) {
        motor.getPIDController().setReference(position, ControlType.kSmartMotion, 0);
    } else {
        motor.disable();
    }
}

  //moves the hood up
//   public void moveUp() { 
//     if(!(atTopPos())){
//       motor.getPIDController().setReference(Constants.hoodSpeed, ControlType.kDutyCycle);
//     }
//   }

  //moves the hood down
//   public void moveDown(){
//     if(!(atBottomPos())){
//       motor.getPIDController().setReference(-1*(Constants.hoodSpeed), ControlType.kDutyCycle);
//     }

//   }

  //returns the position of the hood motor
  public double getPosition(){
    return motor.getEncoder().getPosition();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    positionHood.setDouble(getPosition());
    motor.getPIDController().setP(proportionalValue.getDouble(0), slotID);
    motor.getPIDController().setI(integralValue.getDouble(0), slotID);
    motor.getPIDController().setD(derivativeValue.getDouble(0), slotID);

    motor.getPIDController().setSmartMotionMaxVelocity(maxVel.getDouble(0), slotID);
    motor.getPIDController().setSmartMotionMaxAccel(maxAcc.getDouble(0), slotID);

    Constants.HOOD_UP_SOFT_LIMIT = upLimit.getDouble(0);
  }
}
