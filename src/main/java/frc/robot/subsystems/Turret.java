// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.commands.Turret.*;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  CANSparkMax turretCanSparkMax = new CANSparkMax(Constants.TURRET_MOTOR_PORT, MotorType.kBrushless);
  //DigitalInput turretLimitSwitch = new DigitalInput(10);
  int TurretVelocityPIDSlot = 0;
  int TurretPositionPIDSlot = 1;

  int stopSpeed = 0;
  double leftTurnSpeed = -0.1;
  double rightTurnSpeed = 0.1;

  //Needs accurate values
  double LeftSoftLimit = -55000;
  double RightSoftLimit = 45000;

  double LeftHardLimit = LeftSoftLimit - 4096;
  double RightHardLimit = RightSoftLimit + 4096;
  //Needs testing
  double TurretSmartVelocityP = 0.0005;
  //Tested:
    //0.00001 Doesn't move
    //0.0001 Moves slowly
  double TurretSmartVelocityI = 0;
  double TurretSmartVelocityD = 0;

  double TurretSmartMotionP = 0.0005;
  double TurretSmartMotionI = 0;
  double TurretSmartMotionD = 0.005;

  //P is 0.00001: Doesn't move
    //P is 0.0001:Moves but oscilates
    //P is 0.00002: Moves 
    //P is 0.000025: Moves with less oscilation
    //P is 0.000022: Very little oscilation
    //P is 0.0000211: Much less oscilation but not exact.
    //p is 0.0000221: closer
    //p 0.0000341 works
    //P is 0.0001 works well. May need more increase
    //P is 0.0000341 and D is 0.00019
    //P is 0.0005 and D is 0.005: Error was around 50. Max Velocity was 2000, Max Acceleration is 750 with a few degrees error. There is also less oscilating and the turret itself isn't affected.
  
  double TurretFF = 0;

  double TurretOutputMax = 0.5;
  double TurretOutputMin = -0.5;
  int ConversionFactor = 4096;
  int zeroPosition = 0;

  //Max v(t) and a(t) during Smart Velocity
  double SmartVelocityMaxAcceleration = 750;
  double SmartVelocityMaxVelocity = 2000;

  double SmartMotionMaxAcceleration = 1000;
  double SmartMotionMaxVelocity = 1500;

  //4096 represents 1 revolution
  int TurretPosition = 4096 * 1;

  //Motor is inverted so that when -speed will turn left and +speed will turn right
  boolean InvertMotor = true;

  // NetworkTableEntry ShuffleBoardData = Shuffleboard.getTab("TalonFX").add("P Value", 0.00005).getEntry();
  // ShuffleboardTab TalonFXTab = Shuffleboard.getTab("TalonFX");
  // NetworkTableEntry positionTurret = TalonFXTab.add("Position Value", 0).getEntry();
  // NetworkTableEntry lol1 = TalonFXTab.add("I Value", 0.000000).getEntry();
  // NetworkTableEntry lol2 = TalonFXTab.add("D Value", 0.00019).getEntry();
  // NetworkTableEntry lol3 = TalonFXTab.add("Max Velocity", 2000).getEntry();
  // NetworkTableEntry lol4 = TalonFXTab.add("Max Acceleration", 750).getEntry();
  // NetworkTableEntry lol5 = TalonFXTab.add("Left Limit", -100).getEntry();
  // NetworkTableEntry lol6 = TalonFXTab.add("Right", 100).getEntry();





  /** Creates a new Turret. */
  public Turret() {
    turretCanSparkMax.getEncoder().setPosition(zeroPosition);
    turretCanSparkMax.getPIDController().setReference(0, ControlType.kSmartMotion);
    turretCanSparkMax.getPIDController().setP(TurretSmartVelocityP, TurretVelocityPIDSlot);
    turretCanSparkMax.getPIDController().setI(TurretSmartVelocityI, TurretVelocityPIDSlot);
    turretCanSparkMax.getPIDController().setD(TurretSmartVelocityD, TurretVelocityPIDSlot);

    turretCanSparkMax.getPIDController().setP(TurretSmartMotionP, TurretPositionPIDSlot);
    turretCanSparkMax.getPIDController().setI(TurretSmartMotionI, TurretPositionPIDSlot);
    turretCanSparkMax.getPIDController().setD(TurretSmartMotionD, TurretPositionPIDSlot);

    turretCanSparkMax.getPIDController().setOutputRange(TurretOutputMin, TurretOutputMax);
    turretCanSparkMax.getEncoder().setPositionConversionFactor(ConversionFactor);
    turretCanSparkMax.setInverted(InvertMotor);
    turretCanSparkMax.setIdleMode(IdleMode.kBrake);

    turretCanSparkMax.getPIDController().setSmartMotionMaxAccel(SmartMotionMaxAcceleration, TurretPositionPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxVelocity(SmartMotionMaxVelocity, TurretPositionPIDSlot);

    turretCanSparkMax.getPIDController().setSmartMotionMaxAccel(SmartVelocityMaxAcceleration, TurretVelocityPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxVelocity(SmartVelocityMaxVelocity, TurretVelocityPIDSlot);

    //turretCanSparkMax.setIdleMode(IdleMode.kBrake);
  }
//Position is not used
  public void turretCanTurn(double positionValue){
    if(getPosition() <= RightHardLimit && getPosition() >= LeftHardLimit){
      if(getPosition() <= RightSoftLimit && getPosition() >= LeftSoftLimit){
        turretCanSparkMax.getPIDController().setReference(positionValue, ControlType.kSmartMotion, TurretPositionPIDSlot);
      }
      else if((getPosition() >= Constants.TURRET_TARGET_POSITION) && getPosition() <= RightSoftLimit){
        turretCanSparkMax.getPIDController().setReference(positionValue, ControlType.kSmartMotion, TurretPositionPIDSlot);
      }
      else if((getPosition() <= Constants.TURRET_TARGET_POSITION) && getPosition() >= LeftSoftLimit){
        turretCanSparkMax.getPIDController().setReference(positionValue, ControlType.kSmartMotion, TurretPositionPIDSlot);
      }
    }
    else{
      turretCanSparkMax.disable();
    }
  }
  // public void turretTest(double positionValue)  {
  //   if(RightSoftLimit <= getPosition() || LeftSoftLimit >= getPosition()) {
  //     if(RightHardLimit <= getPosition() || LeftHardLimit >= getPosition()) {
  //       turretCanSparkMax.disable();
  //     }
  //   }
  //   else {
  //     turretCanSparkMax.getPIDController().setReference(positionValue, ControlType.kSmartMotion, TurretPositionPIDSlot);
  //   }
  // }

  public void turretTurnLeft()  {
    if(LeftSoftLimit <= turretCanSparkMax.getEncoder().getPosition()){
      turretCanSparkMax.getPIDController().setReference(leftTurnSpeed, ControlType.kDutyCycle, TurretVelocityPIDSlot);
    }
    else{
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kDutyCycle, TurretVelocityPIDSlot);
    }
  }

  public void turretTurnRight() {
    if(RightSoftLimit >= turretCanSparkMax.getEncoder().getPosition()){
      turretCanSparkMax.getPIDController().setReference(rightTurnSpeed, ControlType.kDutyCycle, TurretVelocityPIDSlot);
    }
    else{
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kDutyCycle, TurretVelocityPIDSlot);
    }
  }

  public void stopTurret() {
    turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kDutyCycle, TurretVelocityPIDSlot);
  }

  //returns true if limitswitch is hits the reflective tape and returns false otherwise
  // public boolean turretLimitSwitchReflected() {
  //   return !turretLimitSwitch.get();
  // }

  public void resetEncoder()  {
    turretCanSparkMax.getEncoder().setPosition(zeroPosition);
  }

  public double getPosition() {
    return turretCanSparkMax.getEncoder().getPosition();
  }

  public double getVelocity() {
    return turretCanSparkMax.getEncoder().getVelocity();
  }
  public void setEncoder(double pos)  {
    turretCanSparkMax.getEncoder().setPosition(pos);
  }

  @Override
  public void periodic()  {
    //System.out.println(getPosition());
    //System.out.println(getVelocity());
    //turretCanSparkMax.getPIDController().setReference(0.1, ControlType.kDutyCycle);
    //System.out.println(" Pos: " + getPosition() + " Error:" + (double)Constants.visionTable.getEntry("Degree").getNumber(Constants.defaultVisionTurretError) + "PIDError" + (getPosition()-turretShoot.target) + " Target:" + turretShoot.target + " Error:" + (turretShoot.target - getPosition()));
    // double PValue = ShuffleBoardData.getDouble(0.0001);
    // double IValue = lol1.getDouble(0.00000);
    // double DValue = lol2.getDouble(0);
    // double MaxVelocity = lol3.getDouble(2000);
    // double MaxAcceleration = lol4.getDouble(750);
    // LeftSoftLimit = lol5.getDouble(-100);
    // RightSoftLimit = lol6.getDouble(100);
    // LeftHardLimit = LeftSoftLimit - 4096;
    // RightHardLimit = RightSoftLimit + 4096;
    
    // turretCanSparkMax.getPIDController().setSmartMotionMaxVelocity(MaxVelocity, TurretPositionPIDSlot);
    // turretCanSparkMax.getPIDController().setSmartMotionMaxAccel(MaxAcceleration, TurretPositionPIDSlot);
    // positionTurret.setDouble(getPosition());
    // Constants.wantedPositionTurret.setDouble(turretShoot.target);
    // Constants.pidError.setDouble((getPosition()-turretShoot.target));
    //Constants.degrees.setDouble((double)Constants.visionTable.getEntry("Degree").getNumber(Constants.defaultVisionTurretError));
    //public static NetworkTableEntry wantedPositionTurret = TalonFXTab.add("Wanted Position", 0).getEntry();

    
    
  }
}
