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

    public Trigger myRightTriggerLight() { // was armAmp
        return new Trigger(()-> (getRightTriggerAxis() > 0.1));
    }

    public Trigger myLeftTriggerLight() { // was armSpeaker
        return new Trigger(()-> (getLeftTriggerAxis() > 0.1));
    }

    public Trigger myRightTriggerHeavy() { // was fireAmp
        return rightTrigger(0.8);
    }

    public Trigger myLeftTriggerHeavy() { // was fireSpeaker
        return leftTrigger(0.8);
    }

    public Trigger myLeftBumper() { // was fpvIntake
        return leftBumper().debounce(0.1);
    }

    public Trigger myRightBumper() { // was intake
        return rightBumper();
    }

    public Trigger myStart() { // was forceFeed
        return start();
    }

    public Trigger myBack() { // was forceReverse
        return back();
    }

    public Trigger myB() { // was shuttle
        return b();
    }

    public Trigger myX() { // was xMode
        return x();
    }

    public Trigger myPOVUp() { // was lUp
        return povUp();
    }

    public Trigger myPOVDown() { // lDown
        return povDown();
    }

    public Trigger myY() { // was rUp
        return y();
    }

    public Trigger myA() { // was rDown
        return a();
    }

    public Trigger myRightStick() { // was resetGyro
        return rightStick();
    }

    public Trigger myLeftAndRightBumper() { // was stow
        return leftBumper().and(rightBumper());
    }

    public Trigger myPOVLeft() { // was homeClimbers
        return povLeft();
    }
}
