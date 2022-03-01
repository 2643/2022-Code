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
