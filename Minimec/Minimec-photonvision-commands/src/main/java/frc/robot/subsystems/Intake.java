package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    /*We need methods to intake and stop when note is detected, feed to shooter, reverse intake and feed manually.
     * Common wisdom says that the intake should run at 2x drive speed.
     */

    private final DigitalInput optical;


    public Intake(int intakeID, int sensorDIO) {

        optical = new DigitalInput(sensorDIO);
    
    }

    public void hold() {

    }

    public void runAtVelocity(double setpoint) {

    }

    public void runOpenLoop(double supplier) {

    }

    public void autoIntake() {

    }

    public boolean isNote() {
        return optical.get();
    }

    
}
