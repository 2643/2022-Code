// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static double TANKDRIVE_SETPOINT = 33000;
    public static final double TANKDRIVE_SLEW_RATE = 1980/TANKDRIVE_SETPOINT;

    public static final double DRIVETRAIN_ALLOWED_ERROR = 2048;
    public static final double DRIVETRAIN_VELOCITY = 22000;
    public static final double DRIVETRAIN_ACCELERATION = 2200;

    public static final int rightClimberPort = 4; //add port num later
    public static final int leftClimberPort = 3; //add port num later

    public static final double gearBoxRatio = 100; //gear box ratio is 100:1 
  
    public static final double downSoftLimit = 0;
    public static final double upSoftLimit = 1250000;
  
    public static final double downHardLimit = -9000;
    public static final double upHardLimit = 1259000;

    public static final double climberGain = 0.5;
    public static final double climberSpeed = 500 * 100; // not really speed, but treat this like it
}
