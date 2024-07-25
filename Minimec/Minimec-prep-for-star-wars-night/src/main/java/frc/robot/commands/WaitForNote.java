
package frc.robot.commands;

import frc.robot.subsystems.PhotonObjects;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** A command that will turn the robot to the specified angle using a motion profile. */
public class WaitForNote extends CommandBase {
  
  private static boolean m_shuffleboardLoaded = false;
  private PhotonObjects m_photon;

  // constructor
  public WaitForNote(PhotonObjects photon) {
    m_photon = photon;
  }

  @Override
  public void execute() {
    m_photon.UpdateTarget();
    super.execute();
  }  

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return m_photon.hasTarget(); // end if we have a target
  }

}
