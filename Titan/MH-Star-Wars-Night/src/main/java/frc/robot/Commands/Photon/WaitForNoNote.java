
package frc.robot.Commands.Photon;

import frc.robot.subsystems.PhotonObjects;
import edu.wpi.first.wpilibj2.command.Command;


/** A command that will turn the robot to the specified angle using a motion profile. */
public class WaitForNoNote extends Command {
  
  private static boolean m_shuffleboardLoaded = false;
  private PhotonObjects m_photon;

  // constructor
  public WaitForNoNote(PhotonObjects photon) {
    super();
    addRequirements(photon);
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
    return !m_photon.hasTarget(); // end if we do NOT have a target
  }

}
