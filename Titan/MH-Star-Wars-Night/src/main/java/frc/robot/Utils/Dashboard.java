package frc.robot.Utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;


public class Dashboard extends SubsystemBase{
    // intend to use elastic
    /* only use accessor methods here except for diagnostics controls
        Diagnostics controls go in subsystem tabs NOT in match tab.
        The match tab should provide a clean end user experiance that delivers only match critical information.
    */

    public Dashboard(
     Arm arm,
     Climber port, 
     Climber starboard
     /*Sendable autochooser*/) {

        Shuffleboard.getTab("drivetrain");
        Shuffleboard.getTab("arm");
        Shuffleboard.getTab("intake");
        Shuffleboard.getTab("shooter");
        Shuffleboard.getTab("climbers");

        Shuffleboard.getTab("match").addBoolean("port homed", () -> port.isHomed());
        Shuffleboard.getTab("match").addBoolean("starboard homed", () -> starboard.isHomed());
        Shuffleboard.getTab("match").addCamera("05", "Microsoft_LifeCam_HD_3000", "10.80.77.56:5800");


        Shuffleboard.getTab("arm").addDouble("arm pos", () -> arm.getPos());
        Shuffleboard.getTab("arm").addDouble("left temp", () -> arm.getTemp()[0]);
        Shuffleboard.getTab("arm").addDouble("right temp", () -> arm.getTemp()[1]);
        Shuffleboard.getTab("arm").addDouble("left current", () -> arm.getCurrent()[0]);
        Shuffleboard.getTab("arm").addDouble("right current", () -> arm.getCurrent()[1]);
            
        Shuffleboard.getTab("climbers").addBoolean("port homed", () -> port.isHomed());
        Shuffleboard.getTab("climbers").addBoolean("starboard homed", () -> starboard.isHomed());
        Shuffleboard.getTab("climbers").addDouble("port pos", () -> port.getPos());
        Shuffleboard.getTab("climbers").addDouble("starboard pos", () -> starboard.getPos());
    }
}
