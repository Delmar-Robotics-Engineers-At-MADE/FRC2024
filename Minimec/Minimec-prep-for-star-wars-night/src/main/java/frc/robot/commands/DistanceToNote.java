// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonObjects;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** A command that will turn the robot to the specified angle using a motion profile. */
public class DistanceToNote extends PIDCommand {
  
  private static PIDController m_PID = new PIDController(
    DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);

  private static boolean m_shuffleboardLoaded = false;
  private PhotonObjects m_photon;

  // constructor
  public DistanceToNote(double targetDistance, PhotonObjects photon, DriveSubsystem drive) {
    super(
        m_PID,
        // Close loop on heading
        photon::getHeight,
        // Set reference to target
        targetDistance,
        // Pipe output to turn robot
        output -> drive.drive(output, 0, 0, false),
        // Require the drive
        drive);

    m_photon = photon;

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kDriveToleranceDist);
      
        // Add the PID to dashboard
      if (!m_shuffleboardLoaded) {
        ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
        turnTab.add("Distance PID", m_PID);
        m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
      }
      System.out.println("new distance to note command created");
  
  }

  @Override
  public void execute() {
    m_photon.UpdateTarget();
    super.execute();
  }  

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint() || !m_photon.hasTarget(); // end if we are at goal, or if we lost target
  }

}
