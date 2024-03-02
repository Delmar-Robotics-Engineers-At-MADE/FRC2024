// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import frc.robot.Constants2.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Photonvision;

public class AutoIntake extends PIDDrive {
  
  private DriveSubsystem drivetrain;
  private Photonvision pCam;
  private boolean end;
  /** Creates a new AutoIntake. */
  public AutoIntake(DriveSubsystem dt, Intake in, Photonvision pv, Arm ar) {
    super(dt);
    pCam = pv;
    end = false;
    drivetrain = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, pv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("MJS: executing AutoIntake");
    if(pCam.isObj()) {
      this.setValues(0, 0, -pCam.objYaw());
      super.execute();
      if(this.atGoal()) {
        System.out.println("MJS: AutoIntake: driving forward");
        drivetrain.drive(OperatorConstants.kManoeuvreSpeed, 0, 0, false, true); 

        // supposed to end when note is in intake
        // if(!intake.isNote()) {
        //   intake.autoIntake();
        // }
        // else {
        //   end = true;
        // }

      }
    }
    else {
      System.out.println("MJS: AutoIntake: lost obj; ending");
      end = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
