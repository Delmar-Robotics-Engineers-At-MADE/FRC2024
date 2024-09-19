package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

// import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

  private double m_bestArea, m_bestX, m_bestY;
  private boolean m_hasTarget = false;
  private NetworkTable m_table = null;
  NetworkTableEntry m_tx = null;
  NetworkTableEntry m_ty = null;
  NetworkTableEntry m_ta = null;
  NetworkTableEntry m_tv = null;

  /* Constructor */
  public LimelightSubsystem() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    m_tx = m_table.getEntry("tx");
    m_ty = m_table.getEntry("ty");
    m_ta = m_table.getEntry("ta");
    m_tv = m_table.getEntry("tv");
    m_bestArea = m_bestX = m_bestY = 0.0;
  }

  // Notes

  public void updateBestTarget() {
    // this using LimelightHelpers, but didn't work at first, so went straight to network table
    // m_bestArea = LimelightHelpers.getTA("limelight");
    // m_bestX = LimelightHelpers.getTX("limelight");
    // m_bestY = LimelightHelpers.getTY("limelight");
    // m_hasTarget = (LimelightHelpers.getTargetCount("limelight") > 0);

    m_hasTarget = (m_tv.getInteger(0) > 0);
    if (m_hasTarget) {
      m_bestArea = m_ta.getDouble(0.0);
      m_bestX = m_tx.getDouble(0.0);
      m_bestY = m_ty.getDouble(0.0);
    }
  }

  public double getArea() { return m_bestArea; }
  public double getX() { return m_bestX; }
  public double getY() { return m_bestY; }
  public boolean hasTarget() { return m_hasTarget; }


}


