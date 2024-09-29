// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utils.Dashboard;
import frc.robot.Utils.Toolkit;
import frc.robot.Utils.Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Commands.Arm.HoldArm;
import frc.robot.Commands.Arm.RunArmClosedLoop;
import frc.robot.Commands.CMDGroup.ForceFeed;
import frc.robot.Commands.CMDGroup.ForceReverse;
import frc.robot.Commands.Climbers.HoldClimber;
import frc.robot.Commands.Climbers.HomeClimber;
import frc.robot.Commands.Climbers.RunClimberNormalLaw;

import frc.robot.Commands.Intake.Feed;
import frc.robot.Commands.Intake.HoldIntake;
import frc.robot.Commands.Intake.IntakeNoteAutomatic;
import frc.robot.Commands.Intake.RunIntakeOpenLoop;
import frc.robot.Commands.Shooter.AccelerateShooter;
import frc.robot.Commands.Shooter.RunShooterEternal;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Arm m_arm = new Arm(ArmConstants.kLeftID, ArmConstants.kRightID);
  private final Blinkin blinkin = new Blinkin();
  private final Intake m_intake = new Intake(IntakeConstants.kIntakeID, IntakeConstants.kPortSensorDIOPort, IntakeConstants.kStarboardSensorDIOPort, blinkin);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kTopID, ShooterConstants.kBottomID);
  private final Climber m_portClimber = new Climber(ClimberConstants.kPortID, ClimberConstants.kPortDIO, true);
  private final Climber m_starboardClimber = new Climber(ClimberConstants.kStarboardID, ClimberConstants.kStarboardDIO, false);

  // The driver's controller
  private Controller c0 = new Controller(0);
  private Controller c1 = new Controller(1);
  //DriverCommandXboxController m_driverControllerX = new DriverCommandXboxController(2);

  private final Dashboard dashboard;


  private boolean override = false;

  // ************* these commands used only by autonomous

  ParallelCommandGroup intake = new ParallelCommandGroup(
    new IntakeNoteAutomatic(m_intake),
    new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)
  );

  // ************* these commands used by teleop

  // intake with lights going grean after success
  SequentialCommandGroup intakeAndSignal = new SequentialCommandGroup(
    new IntakeNoteAutomatic(m_intake),
    new InstantCommand(() ->  blinkin.setColour(LEDConstants.green)) // blinkin.green());
    );

  ParallelCommandGroup homeClimbers = new ParallelCommandGroup(
    new HomeClimber(m_portClimber),
    new HomeClimber(m_starboardClimber)
  );

  ForceFeed forceFeed = new ForceFeed(m_intake, m_shooter);

  ParallelCommandGroup forceReverse = new ParallelCommandGroup(
    new RunIntakeOpenLoop(m_intake, IntakeConstants.kReverseSpeed),
    new RunShooterEternal(m_shooter, ShooterConstants.kAmpSpeed, true)
  );


  // ************* unused commands

  SequentialCommandGroup subwooferFire = new SequentialCommandGroup(
        Toolkit.sout("sFire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos),
          new AccelerateShooter(m_shooter, ShooterConstants.kSubwooferSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.kSubwooferSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("sFire end")
      );
  SequentialCommandGroup distanceFire = new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.k3mPos),
          new AccelerateShooter(m_shooter, ShooterConstants.k3mSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.k3mSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("Fire end")
      );
  SequentialCommandGroup shuttleFire = new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kShuttlePos),
          new AccelerateShooter(m_shooter, ShooterConstants.kShuttleSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.kShuttleSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("Fire end")
      );
  SequentialCommandGroup angleFire = new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kAnglePos),
          new AccelerateShooter(m_shooter, ShooterConstants.kAngleSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.kAngleSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("Fire end")
      );
  SequentialCommandGroup backAmp = new SequentialCommandGroup(
        Toolkit.sout("AMP init"),
          new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos),
          new AccelerateShooter(m_shooter, ShooterConstants.kAmpSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.kAmpSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("Amp end")
      );
  SequentialCommandGroup frontAmp = new SequentialCommandGroup(
        Toolkit.sout("AMP init"),
        new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunIntakeOpenLoop(m_intake, IntakeConstants.kReverseSpeed)
        ),
        Toolkit.sout("Amp end")
      );

  private final SendableChooser<Command> autoChooser;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Register Named Commands
    NamedCommands.registerCommand("intake", intake);
    NamedCommands.registerCommand("shootSubwoofer", new SequentialCommandGroup(
      new ParallelRaceGroup(
      new WaitCommand(4),
        armSpeaker()),
      new ParallelRaceGroup(
        fireSpeaker(),
        new WaitCommand(3))
    ));
    NamedCommands.registerCommand("shootpodium", new SequentialCommandGroup(
        Toolkit.sout("Fire init"),
          new RunArmClosedLoop(m_arm, ArmConstants.k3mPos),
          new AccelerateShooter(m_shooter, ShooterConstants.k3mSpeed),
        Toolkit.sout("shoot init"),
        new ParallelRaceGroup(
          new WaitCommand(ShooterConstants.kLaunchTime),
          new RunShooterEternal(m_shooter, ShooterConstants.k3mSpeed, true),
          new Feed(m_intake)
        ),
        Toolkit.sout("Fire end")
      ));
    NamedCommands.registerCommand("armInside", new RunArmClosedLoop(m_arm, ArmConstants.kStowPos));
    NamedCommands.registerCommand("homePort", new HomeClimber(m_portClimber));
    NamedCommands.registerCommand("homeStarboard", new HomeClimber(m_starboardClimber));
    NamedCommands.registerCommand("armDown", new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos));
    NamedCommands.registerCommand("drivetrainStop", new RunCommand(
      () -> m_robotDrive.drive(
        0, 0, 0, false, true), m_robotDrive));

    // Another option that allows you to specify the default auto by its name
    autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    dashboard = new Dashboard(m_robotDrive, m_arm, m_intake, m_shooter, m_portClimber, m_starboardClimber, autoChooser);
    Shuffleboard.getTab("match").add(autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(

      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(c0.getLeftY()*DriverConstants.kDefaultSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(c0.getLeftX()*DriverConstants.kDefaultSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(c0.getRightX()*DriverConstants.kYawSpeed, 0.05),
          true, true), m_robotDrive));

    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    c0.leftBumper().debounce(0.1).whileTrue(
    new RunCommand(
      () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(c0.getLeftY()*DriverConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(c0.getLeftX()*DriverConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(c0.getRightX()*DriverConstants.kMYawSpeed, 0.05),
        true,true), m_robotDrive));

    c1.leftBumper().debounce(0.1).whileTrue(
    new RunCommand(
      () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(c1.getRightY()*DriverConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(c1.getRightX()*DriverConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(c1.getLeftX()*DriverConstants.kMYawSpeed, 0.05),
        true,true), m_robotDrive));

    c0.leftStick().whileTrue(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(c0.getLeftY()*DriverConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(c0.getLeftX()*DriverConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(c0.getRightX()*DriverConstants.kYawSpeed, 0.05),
          false, true), m_robotDrive));

    c1.leftStick().whileTrue(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(c1.getRightY()*DriverConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(c1.getRightX()*DriverConstants.kManoeuvreSpeed, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(c1.getLeftX()*DriverConstants.kYawSpeed, 0.05),
          false, true), m_robotDrive));

    // Subsystem Default Commands
    //m_intake.setDefaultCommand(new HoldIntake(m_intake));
    m_arm.setDefaultCommand(new SequentialCommandGroup(
      new RunArmClosedLoop(m_arm, ArmConstants.kDefaultPos),
      new HoldArm(m_arm)));
    m_shooter.setDefaultCommand(new RunShooterEternal(m_shooter, ShooterConstants.kIdleSpeed, false));
    m_portClimber.setDefaultCommand(new HoldClimber(m_portClimber));
    m_starboardClimber.setDefaultCommand(new HoldClimber(m_starboardClimber));

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
    c0.myX().or(c1.myX())
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    c0.myLeftBumper().or(c1.myLeftBumper()).whileTrue(new ParallelCommandGroup(
      new IntakeNoteAutomatic(m_intake),
      // intakeAndSignal,
      new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)));
    c0.myRightBumper().or(c1.myRightBumper()).whileTrue(new ParallelCommandGroup(
      new IntakeNoteAutomatic(m_intake),
      new RunArmClosedLoop(m_arm, ArmConstants.kIntakePos)));

    c0.myLeftAndRightBumper().or(c1.myLeftAndRightBumper()).toggleOnTrue(new SequentialCommandGroup(
      new RunArmClosedLoop(m_arm, ArmConstants.kStowPos),
      new HoldArm(m_arm)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    c0.myRightStick().or(c1.myRightStick()).onTrue(
      new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
    );

    c0.myStart().or(c1.myStart()).whileTrue(new ForceFeed(m_intake, m_shooter));
    c0.myBack().or(c1.myBack()).whileTrue(new ForceReverse(m_intake, m_shooter));

    c0.myPOVLeft().or(c1.myPOVLeft()).whileTrue(homeClimbers);
    c0.myY().or(c1.myY()).whileTrue(new RunClimberNormalLaw(m_starboardClimber, true));
    c0.myA().or(c1.myA()).whileTrue(new RunClimberNormalLaw(m_starboardClimber, false));
    c0.myPOVUp().or(c1.myPOVUp()).whileTrue(new RunClimberNormalLaw(m_portClimber, true));
    c0.myPOVDown().or(c1.myPOVDown()).whileTrue(new RunClimberNormalLaw(m_portClimber, false));

    c0.myRightTriggerLight().or(c1.myRightTriggerLight()).whileTrue(new ParallelCommandGroup(
      m_shooter.accelerate(ShooterConstants.kAmpSpeed),
      new RunArmClosedLoop(m_arm, ArmConstants.kBackAmpPos)));

    c0.myLeftTriggerLight().or(c1.myLeftTriggerLight()).whileTrue(new ParallelCommandGroup(
      m_shooter.accelerate(ShooterConstants.kSubwooferSpeed),
      new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos)));


    c0.myRightTriggerHeavy().or(c1.myRightTriggerHeavy()).whileTrue(new ParallelCommandGroup(
      m_intake.feed()
    ));
    c0.myB().or(c1.myB()).whileTrue(fireShuttle());
    c0.myLeftTriggerHeavy().or(c1.myLeftTriggerHeavy()).whileTrue(new ConditionalCommand(
      m_intake.feed(), new HoldIntake(m_intake), m_shooter::isSubwooferSpeed));
  }
    
  private Command fireSpeaker() {
    return
      new ParallelCommandGroup(
      new HoldArm(m_arm),
      m_shooter.fire(ShooterConstants.kSubwooferSpeed),
      m_intake.feed());
  }

  private Command fireShuttle() {
    return
      new ParallelCommandGroup(
      new HoldArm(m_arm),
      m_shooter.fire(ShooterConstants.kShuttleSpeed),
      m_intake.feed());
  }

  private Command armSpeaker() {
    return
    new ParallelCommandGroup(
      m_shooter.accelerate(ShooterConstants.kSubwooferSpeed),
      new RunArmClosedLoop(m_arm, ArmConstants.kSubwooferPos)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Blinkin getBlinkin() {
    return blinkin;
  }
}
