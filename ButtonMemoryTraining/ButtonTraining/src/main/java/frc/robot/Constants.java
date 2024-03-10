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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static enum Functions {
    AMP,
    SHOOT_DISTANCE,
    INTAKE,
    SHOOT_SUBWOOFER,
    STARBOARD_CLIMBER_UP,
    STARBOARD_CLIMBER_DOWN,
    PORT_CLIMBER_UP,
    PORT_CLIMBER_DOWN,
    OVERRIDE,
    HOME_CLIMBERS,
    MANOEUVER_DRIVE,
    ARM_UP,
    FEED,
    ARM_DOWN,
    REVERSE;
  }

  public static final class FunctionList {
    public static final String AMP = "Amp";
    public static final String SHOOT_DISTANCE = "Shoot at Distance";
    public static final String INTAKE = "Intake";
    public static final String SHOOT_SUBWOOFER = "Shoot at SUBWOOFER";
    public static final String STARBOARD_CLIMBER_UP = "Starboard Climber Up";
    public static final String STARBOARD_CLIMBER_DOWN = "Starboard Climber Down";
    public static final String PORT_CLIMBER_UP = "Port Climber Up";
    public static final String PORT_CLIMBER_DOWN = "Port Climber Down";
    public static final String OVERRIDE = "OVERIDE";
    public static final String HOME_CLIMBERS = "Home Climbers";
    public static final String MANOEUVER_DRIVE = "Drive Slow Speed";
    public static final String ARM_UP = "Arm Up";
    public static final String FEED = "Force Feed Note";
    public static final String ARM_DOWN = "Arm Down";
    public static final String REVERSE = "Force Reverse Note";
  }
}
