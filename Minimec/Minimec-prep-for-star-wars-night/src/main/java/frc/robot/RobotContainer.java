// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TurnOrStrafeAndDistance;
import frc.robot.commands.UpdateBestPhotonCommand;
import frc.robot.commands.WaitForNote;
import frc.robot.commands.TurnOrStrafeToNote;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonObjects;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final PhotonObjects m_photon = new PhotonObjects(null);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_photon);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // sequences for summer demos
  private final SequentialCommandGroup m_cmdWaitForNote = new SequentialCommandGroup(
    new InstantCommand(() -> System.out.println("waiting for note")),
    new WaitForNote(m_photon),
    new InstantCommand(() -> System.out.println("done waiting for note"))
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    // m_robotDrive.setDefaultCommand(
    //     // A split-stick arcade command, with forward/backward controlled by the left
    //     // hand, and turning controlled by the right.
    //     new RunCommand(
    //         () ->
    //             m_robotDrive.drive(
    //                 -m_driverController.getLeftY()/2,
    //                 m_driverController.getLeftX()/2,
    //                 m_driverController.getRightX()/2,
    //                 true),
    //         m_robotDrive));

    // default command
    m_robotDrive.setDefaultCommand(m_cmdWaitForNote);

    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    driveBaseTab.add("Mecanum Drive", m_robotDrive);
    driveBaseTab.add("Gyro", m_robotDrive.m_gyro);

    // Put all encoders in a list layout
    ShuffleboardLayout encoders =
        driveBaseTab.getLayout("Encoders", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4);
    encoders.add("Front Left Encoder", m_robotDrive.getFrontLeftEncoder());
    encoders.add("Front Right Encoder", m_robotDrive.getFrontRightEncoder());
    encoders.add("Rear Left Encoder", m_robotDrive.getRearLeftEncoder());
    encoders.add("Rear Right Encoder", m_robotDrive.getRearRightEncoder());

    // Add photon stuff
    ShuffleboardTab photonTab = Shuffleboard.getTab("Photon");
    photonTab.addNumber("Pitch", () -> m_photon.getPitch());
    photonTab.addNumber("Yaw", () -> m_photon.getYaw());
    photonTab.addNumber("Aspect", () -> m_photon.getAspect());
    photonTab.addNumber("Height", () -> m_photon.getHeight());
    photonTab.addNumber("Width", () -> m_photon.getWidth());

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));

    // reset gyro heading when green-A pressed
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // reset encoders when yellow-Y pressed
    // new JoystickButton(m_driverController, Button.kY.value)
    //     .onTrue(new InstantCommand(() -> m_robotDrive.resetEncoders()));

    // update info from photon when yellow-Y pressed
    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new RepeatCommand(new UpdateBestPhotonCommand(m_photon)));

    // distance to note when blue-X pressed
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RepeatCommand(new TurnOrStrafeToNote(m_photon, m_robotDrive)));

    // rotate to note when red-B pressed
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new RepeatCommand(new TurnOrStrafeAndDistance(DriveConstants.kSummerDistance, m_photon, m_robotDrive)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    MecanumControllerCommand mecanumControllerCommand =
        new MecanumControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            DriveConstants.kFeedforward,
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

            // Needed for normalizing wheel speeds
            AutoConstants.kMaxSpeedMetersPerSecond,

            // Velocity PID's
            new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
            new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
            new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
            new PIDController(DriveConstants.kPRearRightVel, 0, 0),
            m_robotDrive::getCurrentWheelSpeeds,
            m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
            m_robotDrive);

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.sequence(
        new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
        mecanumControllerCommand,
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
  }
}
