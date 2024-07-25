    // fronCam = new PhotonCamera( "Microsoft_LifeCam_HD-3000");
    // backCam = new PhotonCamera( "HD_Pro_Webcam_C920");

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonAprilTags extends SubsystemBase {

  private PhotonCamera m_backCam;
  private PhotonPipelineResult m_latestResult;
  private boolean m_hasTargets = false;

  // Constructor
  public PhotonAprilTags(NetworkTableInstance nt) {
    m_backCam = new PhotonCamera(nt, "HD_Pro_Webcam_C920");
  }

  // AprilTags

  public boolean checkHasAprilTagTargets() {
    m_latestResult = m_backCam.getLatestResult();
    m_hasTargets = m_latestResult.hasTargets();
    return m_hasTargets;
  }

  public int getTargetId() {
    int result = -1; // -1 indicates no targets
    if (m_hasTargets) {
      List<PhotonTrackedTarget> targets = m_latestResult.getTargets();
      if (targets.size() > 0) {
        result = targets.get(0).getFiducialId();
      }
    }
    return result;
  }
  
}


