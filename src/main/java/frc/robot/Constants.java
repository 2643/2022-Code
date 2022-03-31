// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int DRIVETRAIN_FRONT_RIGHT_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_RIGHT_MOTOR = 2;
    public static final int DRIVETRAIN_FRONT_LEFT_MOTOR = 3;
    public static final int DRIVETRAIN_BACK_LEFT_MOTOR = 4;

    public static final int JOYSTICK_LEFT_AXIS = 1;
    public static final int JOYSTICK_RIGHT_AXIS = 5;
    public static final double JOYSITCK_DEADBAND = 0.05;
    public static boolean slowMode = false;
    public static final double SLOW_MODE_MULTIPLIER = 0.5;
    
    // Max output set speed point to 66k
    public static double TANKDRIVE_SETPOINT = 66000;
    public static final double TANKDRIVE_SLEW_RATE = 1980/TANKDRIVE_SETPOINT;

    public static final double DRIVETRAIN_ALLOWED_ERROR = 2048;
    public static final double DRIVETRAIN_VELOCITY = 22000;
    public static final double DRIVETRAIN_ACCELERATION = 2200;

    public static final int rightClimberPort = 6; //add port num later
    public static final int leftClimberPort = 5; //add port num later

    public static final double gearBoxRatio = 100; //gear box ratio is 100:1 
  
    public static final double downSoftLimit = 0;
    public static final double upSoftLimit = 1250000;
  
    public static final double downHardLimit = -9000;
    public static final double upHardLimit = 1259000;

    public static final double climberGain = 0.5;
    public static final double climberSpeed = 500 * 100; // not really speed, but treat this like it

    public static final int TurretMotorPort = 7;
    public static final int turretLimitSwitchPort = 0;
    public static final double defaultVisionTurretError = 0;
    public static NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision-movement");
    public static int visionTurretError = 10;
    public static ShuffleboardTab TalonFXTab = Shuffleboard.getTab("TalonFX");
    public static NetworkTableEntry wantedPositionTurret = TalonFXTab.add("Wanted Position", 0).getEntry();
    public static NetworkTableEntry pidError =  TalonFXTab.add("PID Error", 0).getEntry();
    public static NetworkTableEntry degrees = TalonFXTab.add("Degrees", 0).getEntry();
    public static final int motorPort = 8;
    public static final double hoodSpeed = 0.07;

    public static final int maxHoodLimitPort = 12;
    public static final int minHoodLimitPort = 11;
    public static boolean checksForBallChannel;
    public static int intakeBallChannel;
    public static boolean intakesBallChannel;
    public static boolean intakingBall;
}
