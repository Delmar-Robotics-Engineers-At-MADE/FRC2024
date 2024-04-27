    // fronCam = new PhotonCamera( "Microsoft_LifeCam_HD-3000");
    // backCam = new PhotonCamera( "HD_Pro_Webcam_C920");

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {

  private PhotonCamera frontCam;

  private double m_bestYaw, m_bestPitch, m_bestHeight, m_bestWidth, m_bestAspect;
  private boolean m_hasTarget = false;

  /** Creates a new Photonvision. */
  public PhotonSubsystem(NetworkTableInstance nt) {
    frontCam = new PhotonCamera(/* nt, */ "Microsoft_LifeCam_HD-3000");
    frontCam.setPipelineIndex(0);
  }

  public boolean UpdateTarget() {
    var result = frontCam.getLatestResult();
    m_hasTarget = result.hasTargets();
    if (m_hasTarget) {
      PhotonTrackedTarget target = result.getBestTarget();
      m_bestYaw = target.getYaw();
      m_bestPitch = target.getPitch();

      List <TargetCorner> corners = target.getMinAreaRectCorners();
      double top = 0.0, left = 0.0, right = 0.0, bottom = 0.0;
      for (TargetCorner corner : corners) {
        // System.out.println(corner.x + " " + corner.y+ " ");
        top = (top == 0) ? corner.y : Math.max(top, corner.y);
        bottom = (bottom == 0) ? corner.y : Math.min(bottom, corner.y);
        right = (right == 0) ? corner.x : Math.max(right, corner.x);
        left = (left == 0) ? corner.x : Math.min(left, corner.x);
      }
      // System.out.println("top, bottom, right, left: " + top + " " + bottom + " " + right + " " + left);
      m_bestHeight = top - bottom;
      m_bestWidth = right - left;
      m_bestAspect = (m_bestWidth > 0) ? m_bestHeight / m_bestWidth : 0;

      // // give aspect a sign according to which side of center the target is on
      // if (m_bestYaw > 0) {m_bestAspect *= -1;}
    }
    return m_hasTarget;
  }

  public double getYaw() { return m_bestYaw; }
  public double getPitch() { return m_bestPitch; }
  public double getHeight() { return m_bestHeight; }
  public double getWidth() { return m_bestWidth; }
  public double getAspect() { return m_bestAspect; }
  public boolean hasTarget() { return m_hasTarget; }

}