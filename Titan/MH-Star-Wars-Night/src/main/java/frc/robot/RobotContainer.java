package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Utils.Dashboard;
import frc.robot.Utils.Toolkit;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.PhotonObjects;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Commands.Arm.HoldArm;
import frc.robot.Commands.Arm.RunArmClosedLoop;
import frc.robot.Commands.Climbers.HoldClimber;
import frc.robot.Commands.Climbers.HomeClimber;
import frc.robot.Commands.Climbers.RunClimberNormalLaw;
import frc.robot.Commands.Photon.WaitForNoNote;
import frc.robot.Commands.Photon.WaitForNote;
import frc.robot.Utils.Controller;

public class RobotContainer {
  // The robot's subsystems
  private final Arm m_arm = new Arm(ArmConstants.kLeftID, ArmConstants.kRightID);
  private final Climber m_portClimber_unused = new Climber(ClimberConstants.kPortID, ClimberConstants.kPortDIO, true);
  private final Climber m_starboardClimber_unused = new Climber(ClimberConstants.kStarboardID, ClimberConstants.kStarboardDIO, false);
  private final Blinkin blinkin = new Blinkin();
  private final PhotonObjects m_photon = new PhotonObjects(NetworkTableInstance.getDefault());

  // The driver's controller
  private Controller c0 = new Controller(0);

  private final Dashboard dashboard;


  private boolean override = false;

  // Command Groups
  ParallelCommandGroup intake = new ParallelCommandGroup(
    new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)
  );

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    dashboard = new Dashboard(m_arm, m_portClimber_unused, m_starboardClimber_unused /* , autoChooser */);

    // sequences for summer demos
    final SequentialCommandGroup m_cmdWaitForNote = new SequentialCommandGroup(
      new InstantCommand(() -> System.out.println("waiting for note")),
      new WaitForNote(m_photon),
      new RunArmClosedLoop(m_arm, ArmConstants.kSummerOpen),
      new InstantCommand(() -> System.out.println("waiting for NO note")),
      new WaitForNoNote(m_photon),
      new InstantCommand(() -> System.out.println("done waiting for note")),
      new RunArmClosedLoop(m_arm, ArmConstants.kSummerClosed)
    );

    m_cmdWaitForNote.addRequirements(m_photon, m_arm);
    m_arm.setDefaultCommand(new SequentialCommandGroup(m_cmdWaitForNote));
    
    Shuffleboard.getTab("match").addBoolean("override", () -> override);
  }

  private void configureButtonBindings() {
    // c0.intake().whileTrue(new ParallelCommandGroup(
    //   Toolkit.sout("arm intake"),
    //   new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)));

    // c0.stow().toggleOnTrue(new SequentialCommandGroup(
    //   new RunArmClosedLoop(m_arm, ArmConstants.kStowPos),
    //   new HoldArm(m_arm)
    // ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));


    c0.armAmp().whileTrue(new ParallelCommandGroup(
      Toolkit.sout("summer open"),
      new RunArmClosedLoop(m_arm, ArmConstants.kSummerOpen))); // right trigger

    c0.armSpeaker().whileTrue(new ParallelCommandGroup(
      Toolkit.sout("summer closed"),
      new RunArmClosedLoop(m_arm, ArmConstants.kSummerClosed))); // left trigger
  }
    

  public Command getAutonomousCommand() {
    return null; //autoChooser.getSelected();
  }

  public Blinkin getBlinkin() {
    return blinkin;
  }
}
