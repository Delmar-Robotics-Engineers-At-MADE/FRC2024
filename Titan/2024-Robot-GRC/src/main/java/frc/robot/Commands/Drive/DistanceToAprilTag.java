// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonApril;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** A command that will turn the robot to the specified angle using a motion profile. */
public class DistanceToAprilTag extends ProfiledPIDCommand {
  
  private static ProfiledPIDController m_PID = new ProfiledPIDController(
    DriveConstants.kDriveAprilP, DriveConstants.kDriveAprilI, DriveConstants.kDriveAprilD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxMetersPerS,
                DriveConstants.kMaxMetersPerSSquared));

  private static boolean m_shuffleboardLoaded = false;
  private PhotonApril m_photon;
  private DriveSubsystem m_drive;

  // constructor
  public DistanceToAprilTag(double distance, PhotonApril photon, DriveSubsystem drive) {
    super(
        m_PID,
        // Close loop on vision target
        photon::getDistance,
        // Set reference to target: 1 meter
        distance,
        // Pipe output to turn robot
        (output, setpoint) -> drive.drive(output, 0, 0, false, true), 
        // Require the drive
        drive);

    m_photon = photon;
    m_drive = drive;

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(DriveConstants.kDriveAprilToleranceDist);
      
      // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
      turnTab.add("April Distance PID", m_PID);
      m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
    }
    System.out.println("new distance to april command created");
  
  }

  @Override
  public void execute() {
    m_photon.updateBestTag();
    System.out.println("distance to april error: " + m_PID.getPositionError());
    super.execute();
  }  

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    boolean result = getController().atGoal() && m_photon.hasTarget() ; // end if we are at goal; keep trying if no target
    if (result) {
      m_drive.drive(0, 0, 0, false, true);
    }
    return result;
  }

}
