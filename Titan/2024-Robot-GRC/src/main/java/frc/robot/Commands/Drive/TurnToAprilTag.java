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
public class TurnToAprilTag extends ProfiledPIDCommand {
  
  private static ProfiledPIDController m_PID = new ProfiledPIDController(
    DriveConstants.kYawAprilP, DriveConstants.kYawAprilI, DriveConstants.kYawAprilD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxTurnRateDegPerS,
                DriveConstants.kMaxTurnAccelerationDegPerSSquared));

  private static boolean m_shuffleboardLoaded = false;
  private PhotonApril m_photon;
  private int m_tagNum;

  // constructor
  public TurnToAprilTag(int tagNum, PhotonApril photon, DriveSubsystem drive) {
    super(
        m_PID,
        // Close loop on vision target
        photon::getYaw,
        // Set reference to target
        DriveConstants.kYawAprilZero,
        // Pipe output to turn robot
        (output, setpoint) -> drive.drive(0, 0, -output, false, true), 
        // Require the drive
        drive);

    m_photon = photon;
    m_tagNum = tagNum;

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceM, DriveConstants.kTurnRateToleranceMPerS);
      
        // Add the PID to dashboard
      if (!m_shuffleboardLoaded) {
        ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
        turnTab.add("Rotate PID", m_PID);
        m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
      }
      System.out.println("new turn to april command created");
  
  }

  @Override
  public void execute() {
    m_photon.updateBestTag(m_tagNum);
    // System.out.println("just turn to april error: " + m_PID.getPositionError());
    super.execute();
  }  

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal() && m_photon.hasTarget(); // end if we are at goal; keep looking if no target
  }

}
