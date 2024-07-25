// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.unused;

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
public class TurnToNoteAspect extends PIDCommand {
  
  private static PIDController m_PID = new PIDController(
    DriveConstants.kAspectP, DriveConstants.kAspectI, DriveConstants.kAspectD);

  private static boolean m_shuffleboardLoaded = false;
  private PhotonObjects m_photon;

  // constructor
  public TurnToNoteAspect(PhotonObjects photon, DriveSubsystem drive) {
    super(
        m_PID,
        // Close loop on heading
        photon::getAspect,
        // Set reference to target
        0.0,
        // Pipe output to turn robot
        output -> drive.drive(0, 0, output, false),
        // Require the drive
        drive);

    m_photon = photon;

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kToleranceAspect, DriveConstants.kTurnRateToleranceDegPerS);
      
        // Add the PID to dashboard
      if (!m_shuffleboardLoaded) {
        ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
        turnTab.add("Aspect Rotate PID", m_PID);
        m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
      }
      System.out.println("new turn to note command created");
  
  }

  @Override
  public void execute() {
    m_photon.UpdateTarget();
    System.out.println("error: " + m_controller.getPositionError());
    super.execute();
  }  

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    boolean atGoal = getController().atSetpoint();
    boolean lostTarget = !m_photon.hasTarget();
    if (atGoal) {System.out.println("at turn goal");}
    if (lostTarget) {System.out.println("lost turn target");}
    return atGoal || lostTarget; // end if we are at goal, or if we lost target
  }

}
