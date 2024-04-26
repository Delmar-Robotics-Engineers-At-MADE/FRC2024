// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftMotorPort = 1;
    public static final int kRearLeftMotorPort = 0;
    public static final int kFrontRightMotorPort = 3;
    public static final int kRearRightMotorPort = 2;

    public static final int[] kFrontLeftEncoderPorts = new int[] {2, 3};
    public static final int[] kRearLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kFrontRightEncoderPorts = new int[] {6, 7};
    public static final int[] kRearRightEncoderPorts = new int[] {4, 5};

    public static final boolean kFrontLeftEncoderReversed = false;
    public static final boolean kRearLeftEncoderReversed = false;
    public static final boolean kFrontRightEncoderReversed = true;
    public static final boolean kRearRightEncoderReversed = true;

    public static final double kTrackWidth = 0.33;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.32;
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderCPR = 74;
    public static final double kWheelDiameterMeters = 0.10;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.5;
    public static final double kPRearLeftVel = 0.5;
    public static final double kPFrontRightVel = 0.5;
    public static final double kPRearRightVel = 0.5;

    // turn to Note
    public static final double kTurnP = 0.035;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0.001;    
    public static final double kMaxTurnRateDegPerS = 180;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;
    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    // distance to Note
    public static final double kDriveP = 0.006;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0001;
    public static final double kDriveToleranceDist = 10;
    public static final double kDriveSetpoint = 300;

    // strafe to Note
    public static final double kStrafeP = 0.7;
    public static final double kStrafeI = 0.0;
    public static final double kStrafeD = 0.0;
    public static final double kStrafeToleranceAspect = 0.1;


  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
