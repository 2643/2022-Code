// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    //DRIVETRAIN
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
    public static final double DRIVETRAIN_VELOCITY = 22000;//22000
    public static final double DRIVETRAIN_ACCELERATION = 5000;

    public static final double DRIVETRAIN_GEARBOX_RATIO = 11.25;


    //CLIMBER
    public static final int RIGHT_CLIMBER_PORT = 5; 
    public static final int LEFT_CLIMBER_PORT = 6; 

    public static final int RIGHT_CLIMB_LIMIT_SWITCH_PORT = 1;
    public static final int LEFT_CLIMB_LIMIT_SWITCH_PORT = 0;
    
    public static final double CLIMBER_GEARBOX_RATIO = 100; 
  
    public static final double DOWN_SOFT_LIMIT_CLIMB_LEFT = 0;
    public static final double UP_SOFT_LIMIT_CLIMB_LEFT = 1390060*20;

    public static final double DOWN_SOFT_LIMIT_CLIMB_RIGHT = 0;
    public static final double UP_SOFT_LIMIT_CLIMB_RIGHT = 1373726*20;
  
    public static final double DOWN_HARD_LIMIT_CLIMB_RIGHT = -9000;
    public static final double UP_HARD_LIMIT_CLIMB_RIGHT = 2048+1373726*20;

    public static final double DOWN_HARD_LIMIT_CLIMB_LEFT = -9000;
    public static final double UP_HARD_LIMIT_CLIMB_LEFT = 2048+1390060*20;

    public static final double CLIMBER_GAIN = 0.5;
    public static final double CLIMBER_SPEED = 500 * 100; // not really speed, but treat this like it


    //AUTONOMOUS
    public static final double MOVE_POSITION_AUTONOMOUS_TO_GET_POINTS = 2048*3.503*DRIVETRAIN_GEARBOX_RATIO;
    public static final double MOVE_BACK_POSITION_TO_SHOOT = 2048*-1*DRIVETRAIN_GEARBOX_RATIO;
    public static int ROUTINE_SELECTOR = 1;
    public static double AUTONOMOUS_DELAY = 0;
    public static final double AUTONOMOUS_SHOOTER_SPEED = 0.625;


    //TURRET
    public static final int TURRET_MOTOR_PORT = 7;
    public static double TURRET_TARGET_POSITION = 0;


    //HOOD
    // public static final int motorPort = 8;
    // public static final double hoodSpeed = 0.07;

    // public static final int MAX_HOOD_LIMIT_PORT = 12;
    // public static final int MIN_HOOD_LIMIT_PORT = 11;

    //CONVEYOR
    
    public static final int CONVEYER_IR_SENSOR_PORT1 = 2;
    public static final int CONVEYER_IR_SENSOR_PORT2 = 3;

    public static final int CONVEYOR_BELT_MOTOR_PORT = 9;

    public static final double CONVEYOR_MOTOR_SPEED = 0.9;
    public static final double CONVEYOR_REVERSE_MOTOR_SPEED = -0.4;

    //INTAKE
    public static int INTAKE_MOTOR_PORT = 10;
    public static final double FORWARD_INTAKE_SPEED = 0.9;
    public static final double REVERSE_INTAKE_SPEED = -0.9;


    // INTAKE LATCH SERVOS
    public static int INTAKE_RIGHT_SERVO_CHANNEL = 0;
    public static int INTAKE_LEFT_SERVO_CHANNEL = 1;

    // Change this also
    public static double INTAKE_SERVO_LATCH_DEGREES = 100;

    
    
    
    //SHOOTER
    public static final double SHOOTER_UP_SPEED = 0.15;
    public static final int LEFT_SHOOTER_PORT = 11;
    public static final int RIGHT_SHOOTER_PORT = 12;
    public static final double CLOSE_SHOOTER_SPEED = 0.5;
    public static final double MEDIUM_SHOOTER_SPEED = 0.7;
    public static final double FAR_SHOOTER_SPEED = 0.9;
    

}
