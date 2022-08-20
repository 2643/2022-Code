// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Hood extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(Constants.HOOD_MOTOR_PORT, MotorType.kBrushless);
  //private DigitalInput maxPositionTrue = new DigitalInput(Constants.maxHoodLimitPort);

  //sets PID variables
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0;
    private static final double FF = 0.000005;

    private static final double outputRange = 0.5;
    private static final int slotID = 0;

    private static final double smartMotionMaxVel = 1500;
    //private static final double smartMotionMaxAccel = 0;
    private static final double smartMotionClosedLoopError = 4096;

    private static final double posConversionFactor = 4096;

    private static final boolean isInverted = true;

    //max/min position that the hood can move to
    //private static final double maxPosition = 10000;
    private static final double minPosition = 0;
    //ShuffleboardTab HoodTab = Shuffleboard.getTab("HoodTab");
    NetworkTableEntry positionHood = Constants.Tab2022.add("Hood Current Pos", 0).withSize(2, 2).getEntry();
    // NetworkTableEntry proportionalValue = HoodTab.add("P Value", 0.0001).getEntry();
    // NetworkTableEntry integralValue = HoodTab.add("I Value", 0).getEntry();
    // NetworkTableEntry feedForwardValue = HoodTab.add("Feed Forward Value", 0.000005).getEntry();
    // NetworkTableEntry derivativeValue = HoodTab.add("D Value", 0).getEntry();
    // NetworkTableEntry maxVel = HoodTab.add("Max Velocity", 1500).getEntry();
    // NetworkTableEntry maxAcc = HoodTab.add("Max Acceleration", 500).getEntry();
    // NetworkTableEntry upLimit = HoodTab.add("Up Limit", 140000).getEntry();
    NetworkTableEntry targetPos = Constants.Tab2022.add("Hood Target Pos", 0).withSize(2, 2).getEntry();



  /** Creates a new Hood. */
  public Hood() {
    //sets PID values
    motor.restoreFactoryDefaults();
    motor.getEncoder().setPositionConversionFactor(posConversionFactor);
    motor.getEncoder().setPosition(0);
    
    motor.getPIDController().setP(kP, slotID);
    motor.getPIDController().setI(kI, slotID);
    motor.getPIDController().setD(kD, slotID);
    motor.getPIDController().setFF(FF, slotID);

    motor.getPIDController().setOutputRange(-1*(outputRange), outputRange);
    motor.getPIDController().setSmartMotionMaxVelocity(smartMotionMaxVel, slotID);
    //motor.getPIDController().setSmartMotionMaxAccel(smartMotionMaxAccel, slotID);
    motor.getPIDController().setSmartMotionAllowedClosedLoopError(smartMotionClosedLoopError, slotID);

    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(isInverted);

    hoodPID(10000);
    motor.getEncoder().setPosition(0);
    hoodPID(0);
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
    hoodPID(targetPos.getDouble(0));
    /*
    if(motor.getPIDController().getP() != proportionalValue.getDouble(0)) {
      motor.getPIDController().setP(proportionalValue.getDouble(0), slotID);
    }
    if(motor.getPIDController().getI() != integralValue.getDouble(0)) {
      motor.getPIDController().setI(integralValue.getDouble(0), slotID);
    }
    if(motor.getPIDController().getD() != derivativeValue.getDouble(0)) {
      motor.getPIDController().setD(derivativeValue.getDouble(0), slotID);
    }
    if(motor.getPIDController().getSmartMotionMaxVelocity(slotID) != maxVel.getDouble(0)) {
      motor.getPIDController().setSmartMotionMaxVelocity(maxVel.getDouble(0), slotID);
    }
    if(motor.getPIDController().getSmartMotionMaxAccel(slotID) != maxAcc.getDouble(0)){
      motor.getPIDController().setSmartMotionMaxAccel(maxAcc.getDouble(0), slotID);
    }
    if(motor.getPIDController().getFF() != feedForwardValue.getDouble(0)){
      motor.getPIDController().setFF(feedForwardValue.getDouble(0), slotID);
    }

    Constants.HOOD_UP_SOFT_LIMIT = upLimit.getDouble(0);
    */
    
  }
}