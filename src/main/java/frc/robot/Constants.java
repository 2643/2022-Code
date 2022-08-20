// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
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
    public static final int LEFT_SHOOTER_PORT = 11;
    public static final int RIGHT_SHOOTER_PORT = 12;

    //NEED TO CONFIGURE
    public static final double SHOOTER_UP_SPEED = 0.15;
    public static final double CLOSE_SHOOTER_SPEED = 0.7;
    public static final double FAR_SHOOTER_SPEED = 0.9;
    public static final double MEDIUM_SHOOTER_SPEED = 0.8;

    public static final int CONVEYER_IR_SENSOR_PORT1 = 2;
    public static final int CONVEYER_IR_SENSOR_PORT2 = 3;

    public static final int CONVEYOR_BELT_MOTOR_PORT = 9;

    public static final double CONVEYOR_MOTOR_SPEED = 0.9;
    public static final double CONVEYOR_REVERSE_MOTOR_SPEED = -0.4;

    public static final int HOOD_MOTOR_PORT = 8;
    public static final double HOOD_SPEED = 0.07;
    
    public static final double HOOD_UP_SOFT_LIMIT = 140000;
    public static final double HOOD_DOWN_SOFT_LIMIT = 0;
    public static final double HOOD_UP_HARD_LIMIT = HOOD_UP_SOFT_LIMIT + 4096;
    public static final double HOOD_DOWN_HARD_LIMIT = HOOD_DOWN_SOFT_LIMIT - 4096;

    public static ShuffleboardTab Tab2022 = Shuffleboard.getTab("2022-Robot");

    public class Tab2022 {
    }


}
