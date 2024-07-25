// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Commands.Arm.HoldArm;
import frc.robot.Commands.Arm.RunArmClosedLoop;
import frc.robot.Commands.Climbers.HoldClimber;
import frc.robot.Commands.Climbers.HomeClimber;
import frc.robot.Commands.Climbers.RunClimberNormalLaw;
import frc.robot.Commands.Photon.WaitForNoNote;
import frc.robot.Commands.Photon.WaitForNote;
import frc.robot.Utils.Controller;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Arm m_arm = new Arm(ArmConstants.kLeftID, ArmConstants.kRightID);
  private final Climber m_portClimber_unused = new Climber(ClimberConstants.kPortID, ClimberConstants.kPortDIO, true);
  private final Climber m_starboardClimber_unused = new Climber(ClimberConstants.kStarboardID, ClimberConstants.kStarboardDIO, false);
  private final Blinkin blinkin = new Blinkin();
  private final PhotonObjects m_photon = new PhotonObjects(null);

  // The driver's controller
  private Controller c0 = new Controller(0);

  private final Dashboard dashboard;


  private boolean override = false;

  // Command Groups
  ParallelCommandGroup intake = new ParallelCommandGroup(
    new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)
  );

  // ParallelCommandGroup homeClimbers = new ParallelCommandGroup(
  //   new HomeClimber(m_portClimber),
  //   new HomeClimber(m_starboardClimber)
  // );

  // Firing Sequences
  SequentialCommandGroup subwooferFire = new SequentialCommandGroup(
        Toolkit.sout("sFire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos),
        Toolkit.sout("shoot init"),
        Toolkit.sout("sFire end")
      );

  SequentialCommandGroup distanceFire = new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.k3mPos),
        Toolkit.sout("shoot init"),
        Toolkit.sout("Fire end")
      );

  SequentialCommandGroup shuttleFire = new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kShuttlePos),
        Toolkit.sout("shoot init"),
        Toolkit.sout("Fire end")
      );


  SequentialCommandGroup angleFire = new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kAnglePos),
        Toolkit.sout("shoot init"),
        Toolkit.sout("Fire end")
      );

  SequentialCommandGroup backAmp = new SequentialCommandGroup(
        Toolkit.sout("AMP init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos),
        Toolkit.sout("shoot init"),
        Toolkit.sout("Amp end")
      );

  SequentialCommandGroup frontAmp = new SequentialCommandGroup(
        Toolkit.sout("AMP init"),
        new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos),
        Toolkit.sout("shoot init"),
        Toolkit.sout("Amp end")
      );

  // private final SendableChooser<Command> autoChooser;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Register Named Commands
    // NamedCommands.registerCommand("intake", intake);
    // NamedCommands.registerCommand("shootSubwoofer", new SequentialCommandGroup(
    //   new ParallelRaceGroup(
    //   new WaitCommand(4),
    //     armSpeaker()),
    //   new ParallelRaceGroup(
    //     fireSpeaker(),
    //     new WaitCommand(3))
    // ));
    // NamedCommands.registerCommand("shootpodium", new SequentialCommandGroup(
    //     Toolkit.sout("Fire init"),
    //       new RunArmClosedLoop(m_arm, ArmConstants.k3mPos),
    //     Toolkit.sout("shoot init"),
    //     Toolkit.sout("Fire end")
    //   ));
    // NamedCommands.registerCommand("armInside", new RunArmClosedLoop(m_arm, ArmConstants.kStowPos));
    // NamedCommands.registerCommand("armDown", new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    dashboard = new Dashboard(m_arm, m_portClimber_unused, m_starboardClimber_unused /* , autoChooser */);
    // Shuffleboard.getTab("match").add(autoChooser);

    // Configure default commands

    // Subsystem Default Commands
    // m_arm.setDefaultCommand(new SequentialCommandGroup(new HoldArm(m_arm)));
    // m_arm.setDefaultCommand(new SequentialCommandGroup( new RunArmClosedLoop(m_arm, ArmConstants.kDefaultPos),m_portClimber.setDefaultCommand(new HoldClimber(m_portClimber));
    // m_starboardClimber.setDefaultCommand(new HoldClimber(m_starboardClimber));

    // sequences for summer demos
    final SequentialCommandGroup m_cmdWaitForNote = new SequentialCommandGroup(
      new InstantCommand(() -> System.out.println("waiting for note")),
      new WaitForNote(m_photon),
      new InstantCommand(() -> System.out.println("waiting for NO note")),
      new WaitForNoNote(m_photon),
      new InstantCommand(() -> System.out.println("done waiting for note"))
    );

    m_cmdWaitForNote.addRequirements(m_photon, m_arm);
    m_arm.setDefaultCommand(new SequentialCommandGroup(m_cmdWaitForNote));
    
    Shuffleboard.getTab("match").addBoolean("override", () -> override);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    c0.intake().whileTrue(new ParallelCommandGroup(
      Toolkit.sout("arm intake"),
      new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)));

    c0.stow().toggleOnTrue(new SequentialCommandGroup(
      new RunArmClosedLoop(m_arm, ArmConstants.kStowPos),
      new HoldArm(m_arm)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // c0.homeClimbers().or(c1.homeClimbers()).whileTrue(homeClimbers);
    // c0.rUp().whileTrue(new RunClimberNormalLaw(m_starboardClimber, true));
    // c0.rDown().whileTrue(new RunClimberNormalLaw(m_starboardClimber, false));
    // c0.lUp().or(c1.lUp()).whileTrue(new RunClimberNormalLaw(m_portClimber, true));
    // c0.lDown().or(c1.lDown()).whileTrue(new RunClimberNormalLaw(m_portClimber, false));

    c0.armAmp().whileTrue(new ParallelCommandGroup(
      Toolkit.sout("arm amp"),
      new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos)));

    c0.armSpeaker().whileTrue(new ParallelCommandGroup(
      Toolkit.sout("arm speaker"),
      new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos)));
  }
    
  private Command fireSpeaker() {
    return
      new ParallelCommandGroup(
      new HoldArm(m_arm));
  }

  private Command fireShuttle() {
    return
      new ParallelCommandGroup(
      new HoldArm(m_arm));
  }

  private Command armSpeaker() {
    return
    new ParallelCommandGroup(
      new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null; //autoChooser.getSelected();
  }

  public Blinkin getBlinkin() {
    return blinkin;
  }
}
