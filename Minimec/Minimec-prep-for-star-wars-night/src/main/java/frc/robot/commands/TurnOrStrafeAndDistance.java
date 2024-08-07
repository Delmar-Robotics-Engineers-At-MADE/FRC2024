// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.PIDBase.ProfiledDoublePIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonObjects;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** A command that will turn the robot to the specified angle using a motion profile. */
public class TurnOrStrafeAndDistance extends ProfiledDoublePIDCommand {
  
  // turn or strafe
  private static ProfiledPIDController m_PID1 = new ProfiledPIDController(
    DriveConstants.kYawP, DriveConstants.kYawI, DriveConstants.kYawD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxTurnRateDegPerS,
                DriveConstants.kMaxTurnAccelerationDegPerSSquared));

  // distance
  private static ProfiledPIDController m_PID2 = new ProfiledPIDController(
    DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxHeightPerS,
                DriveConstants.kMaxHeightPerSSquared));


  private static boolean m_shuffleboardLoaded = false;
  private PhotonObjects m_photon;

  // constructor
  public TurnOrStrafeAndDistance(double targetDistance, PhotonObjects photon, DriveSubsystem drive) {
    super(
        m_PID1, m_PID2,
        // Close loop on heading and distance
        photon::getYaw, photon::getHeight,
        // Set points
        0.0, targetDistance,
        // Pipe output to turn robot
        (output, setpoint) -> drive.turnOrStrafe(output.x2, -output.x1), 
        // Require the drive
        drive);

    m_photon = photon;

    // Set the controller to be continuous (because it is an angle controller)
    getController1().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController1()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
    getController2()
        .setTolerance(DriveConstants.kDriveToleranceDist);
            
        // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
        ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
        turnTab.add("double PID1", m_PID1);
        turnTab.add("double PID2", m_PID2);
        m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
    }
    System.out.println("new turn/strafe/drive to note command created");
  
  }

  @Override
  public void execute() {
    m_photon.UpdateTarget();
    super.execute();
  }  

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return (getController1().atGoal() && getController2().atGoal()) || !m_photon.hasTarget(); // end if we are at goal, or if we lost target
  }

}
