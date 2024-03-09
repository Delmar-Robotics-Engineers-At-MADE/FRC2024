package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants2.ShooterConstants;

public class Shooter extends SubsystemBase{
    /* We should use velocity control to ensure consistant performance.
     * An idle mode for default will help with faster acceleration.
     * A low speed is also needed for AMP.
     */

    public Shooter(int topRollerID, int bottomRollerID) {
    }

    public void runAtSpeed(double target) {
        double setpoint = target * ShooterConstants.kCompenstion;
        //System.out.println(setpoint + " " + top.getOutputCurrent());
    }

    public void runOpenLoop(double speed) {
    }
    public void stop() {
    }

    public double getTopVelocity() {
        return 0.0;
    }

    public double getBottomVelocity() {
        return 0.0;
    }

    public double getAvgVelocity() {
        return (getTopVelocity() + getBottomVelocity())/2;
    }

    public double[] getOutputCurrent() {
        double result[] = {0.0, 0.0};
        return result;
    }

    public double[] getTemp() {
        double result[] = {0.0, 0.0};
        return result;
    }

    public boolean isSafeTemp() {
            return true;
    }
    
}
