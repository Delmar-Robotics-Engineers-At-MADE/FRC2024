// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Controller extends CommandXboxController {

    public Controller(int id) {
        super(id);
    }

    public Trigger armAmp() {
        return new Trigger(()-> (getRightTriggerAxis() > 0.1));
    }

    public Trigger armSpeaker() {
        return new Trigger(()-> (getLeftTriggerAxis() > 0.1));
    }

    public Trigger fireAmp() {
        return rightTrigger(0.8);
    }

    public Trigger fireSpeaker() {
        return leftTrigger(0.8);
    }

    public Trigger fpvIntake() {
        return leftBumper().debounce(0.1);
    }

    public Trigger intake() {
        return rightBumper();
    }

    public Trigger forceFeed() {
        return start();
    }

    public Trigger forceReverse() {
        return back();
    }

    public Trigger shuttle() {
        return b();
    }

    public Trigger xMode() {
        return x();
    }

    public Trigger lUp() {
        return povUp();
    }

    public Trigger lDown() {
        return povDown();
    }

    public Trigger rUp() {
        return y();
    }

    public Trigger rDown() {
        return a();
    }

    public Trigger resetGyro() {
        return y().and(rightBumper());
    }

    public Trigger stow() {
        return leftBumper().and(rightBumper());
    }

    public Trigger homeClimbers() {
        return povLeft();
    }
}
