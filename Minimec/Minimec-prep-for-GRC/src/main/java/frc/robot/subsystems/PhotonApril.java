package frc.robot.subsystems;

// import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonApril extends SubsystemBase {

  private PhotonCamera backCam;
  // private Transform3d fake = new Transform3d();

  private double m_bestDistance, m_bestYaw, m_bestSkew;
  private boolean m_hasTarget = false;

  /* Constructor */
  public PhotonApril(NetworkTableInstance nt) {
    backCam = new PhotonCamera(/* nt, */ "Arducam_OV9281_USB_Camera");
    m_bestDistance = m_bestYaw = m_bestSkew = 0.0;
  }

  // AprilTags

  public void updateBestTag() {
    updateBestTag(-1);
  }

  public void updateBestTag(int id) {
    var result = backCam.getLatestResult();
    m_hasTarget = result.hasTargets();
    if (m_hasTarget) {
      PhotonTrackedTarget target = result.getBestTarget();
      Transform3d target3d = null;
      if (target != null && (id == -1 || target.getFiducialId() == id)) { // pass -1 to accept any id
        target3d = target.getBestCameraToTarget();
      } else {
        // MJS: transform with all zeros
        target3d = new Transform3d(new Translation3d(0, new Rotation3d(0,0,0)), new Rotation3d(0,0,0));
      }
      m_bestDistance = target3d.getX();
      m_bestYaw = target3d.getY();
      m_bestSkew = target3d.getRotation().getAngle();
    }
  }

  public double getDistance() { return m_bestDistance; }
  public double getYaw() { return m_bestYaw; }
  public double getSkew() { return m_bestSkew; }
  public boolean hasTarget() { return m_hasTarget; }


  // public double[] getTagData(int id) {
  //   var result = backCam.getLatestResult();
  //   boolean hasTargets = result.hasTargets();
  //   if (hasTargets) {
  //     PhotonTrackedTarget target = findCorrectTarget(id, result.getTargets());
  //     if (target.getFiducialId() == id) {
  //       double[] pack = {
  //         target.getBestCameraToTarget().getX(), 
  //         target.getBestCameraToTarget().getZ(),
  //         target.getBestCameraToTarget().getZ()
  //       };
  //       return pack;
  //     }
  //     else {
  //       double[] mt = {0,0,0}; // MJS: return all 3 numbers for dashboard
  //       return mt;
  //     }
  //   } else {
  //     double[] mt = {0,0,0}; // MJS: return all 3 numbers for dashboard
  //     return mt;
  //   }
  // }

  // public double[] getStageTagData(int[] ids) {
  //   var result = backCam.getLatestResult();
  //   boolean hasTargets = result.hasTargets();
  //   if (hasTargets) {
  //     PhotonTrackedTarget target = findCorrectTarget(ids, result.getTargets());
  //     if (!(target.getFiducialId() == -1)) {
  //       double[] pack = {getTargetValues().getX(),
  //         getTargetValues().getZ(),
  //         getTargetValues().getRotation().getAngle()};
  //       return pack;
  //     }
  //     else {
  //       double[] mt = {0,0,0}; // MJS: return all 3 numbers for dashboard
  //       return mt;
  //   }
  //   }
  //   else {
  //     double[] mt = {0,0,0}; // MJS: return all 3 numbers for dashboard
  //     return mt;
  //   }
  // }

  // public double[] get3DTagData(int id) {
  //   var result = backCam.getLatestResult();
  //   boolean hasTargets = result.hasTargets();
  //   if (hasTargets) {
  //     PhotonTrackedTarget target = findCorrectTarget(id, result.getTargets());
  //     if (!(target.getFiducialId() == -1)) {
  //       double[] pack = {getTargetValues().getX(),
  //         getTargetValues().getY(),
  //         getTargetValues().getRotation().getAngle()};
  //       return pack;
  //     }
  //     else {
  //       double[] mt = {0,0,0}; // MJS: return all 3 numbers for dashboard
  //       return mt;
  //   }
  //   }
  //   else {
  //     double[] mt = {0,0,0}; // MJS: return all 3 numbers for dashboard
  //     return mt;
  //   }
  // }

  // public Transform3d getTargetValues() {
  //   var result = backCam.getLatestResult();
  //   // boolean hasTargets = result.hasTargets();

  //   PhotonTrackedTarget target = result.getBestTarget();
  //   Transform3d targetVal = null;
  //   if (target != null) {
  //     targetVal = target.getBestCameraToTarget();
  //   } else {
  //     // MJS: return transform with all zeros
  //     targetVal = new Transform3d(new Translation3d(0, new Rotation3d(0,0,0)), new Rotation3d(0,0,0));
  //   }
  //   return targetVal;
  // }


  // public double[] getObjData() {
  //   double data[] = {objYaw(), objPitch(), objSkew()};
  //   return data;
  // }

  // public boolean isTarget(int targetID) {
  //   var result = backCam.getLatestResult();
  //   boolean hasTargets = result.hasTargets();
  //   if (hasTargets) {
  //     PhotonTrackedTarget target = result.getBestTarget();
  //     if (targetID == 0) {
  //       return true;
  //     } else {
  //       if (targetID == target.getFiducialId()) {
  //         return true;
  //       } else {
  //         return false;
  //       }
  //     }
  //   } else {
  //     return false;
  //   }
  // }

  // private PhotonTrackedTarget findCorrectTarget(int id, List<PhotonTrackedTarget> lstTarget) {
  //   for(int x = 0; x<lstTarget.size(); x++) {
  //     if(lstTarget.get(x).getFiducialId() == id) {
  //       return lstTarget.get(x);
  //     }
  //   }
  //   return new PhotonTrackedTarget(0, 0, 0, 0, -1, fake, fake, 0.0, null, null);
  // }

  // private PhotonTrackedTarget findCorrectTarget(int[] ids, List<PhotonTrackedTarget> lstTarget) {
  //   for(int x = 0; x<lstTarget.size(); x++) {
  //     if(lstTarget.get(x).getFiducialId() == ids[0] || lstTarget.get(x).getFiducialId() == ids[1] || lstTarget.get(x).getFiducialId() == ids[2]) {
  //       return lstTarget.get(x);
  //     }
  //   }
  //   return new PhotonTrackedTarget(0, 0, 0, 0, -1, fake, fake, 0.0, null, null);
  // }

}


