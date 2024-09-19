// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.PIDBase.ProfiledDoublePIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** A command that will turn the robot to the specified angle using a motion profile. */
public class TurnAndDistanceLimelight extends ProfiledDoublePIDCommand {
  
  // turn
  private static ProfiledPIDController m_PID1 = new ProfiledPIDController(
    DriveConstants.kYawLimeP, DriveConstants.kYawLimeI, DriveConstants.kYawLimeD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxTurnRateDegPerS,
                DriveConstants.kMaxTurnAccelerationDegPerSSquared));

  // distance
  private static ProfiledPIDController m_PID2 = new ProfiledPIDController(
    DriveConstants.kDriveLimeP, DriveConstants.kDriveLimeI, DriveConstants.kDriveLimeD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxHeightPerS,
                DriveConstants.kMaxHeightPerSSquared));


  private static boolean m_shuffleboardLoaded = false;
  private LimelightSubsystem m_limelight;

  // constructor
  public TurnAndDistanceLimelight(double targetArea , LimelightSubsystem limelight, DriveSubsystem drive) {
    super(
        m_PID1, m_PID2,
        // Close loop on heading and distance
        limelight::getX, limelight::getArea,
        // Set points
        0.0, targetArea,
        // Pipe output to turn robot
        (output, setpoint) -> drive.drive(output.x2, 0, -output.x1, false), 
        // Require the drive
        drive);

    m_limelight = limelight;

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController1()
        .setTolerance(DriveConstants.kTurnToleranceLime, DriveConstants.kTurnRateToleranceLimesPerS);
    getController2()
        .setTolerance(DriveConstants.kDriveToleranceLimeArea);
            
        // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
        ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
        turnTab.add("lime double PID1", m_PID1);
        turnTab.add("lime double PID2", m_PID2);
        m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
    }
    System.out.println("new turn/drive to note (limelight) command created");
  
  }

  @Override
  public void execute() {
    m_limelight.updateBestTarget();
    super.execute();
  }  

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return (getController1().atGoal() && getController2().atGoal()) || !m_limelight.hasTarget(); // end if we are at goal, or if we lost target
  }

}
