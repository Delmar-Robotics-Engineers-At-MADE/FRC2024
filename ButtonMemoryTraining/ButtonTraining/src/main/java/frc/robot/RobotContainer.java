// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final StringXboxController controller =
      new StringXboxController(OperatorConstants.kDriverControllerPort);
  private final Scorekeeper scorekeeper = new Scorekeeper();
  private final Test test = new Test(controller, scorekeeper);
  private final CommandJoystick keyboard = new CommandJoystick(5);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    keyboard.button(1).onTrue(
      new ParallelDeadlineGroup(new WaitCommand(150),
      new ParallelRaceGroup(
        new WaitCommand(3),
        test.activate()
      ).repeatedly()
     ));
  }

}
