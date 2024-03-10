package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FunctionList;

public class StringXboxController extends CommandXboxController{

    public StringXboxController(int id) {
        super(id);
    }

    public String getActiveFunction() {
        if(a().getAsBoolean()){
            return FunctionList.AMP;
        }
        else if(b().getAsBoolean()){
            return FunctionList.SHOOT_DISTANCE;
        }
        else if(x().getAsBoolean()){
            return FunctionList.INTAKE;
        }
        else if(y().getAsBoolean()){
            return FunctionList.SHOOT_SUBWOOFER;
        }
        else if(rightBumper().getAsBoolean()){
            return FunctionList.STARBOARD_CLIMBER_UP;
        }
        else if(rightTrigger().getAsBoolean()){
            return FunctionList.STARBOARD_CLIMBER_DOWN;
        }
        else if(leftTrigger().getAsBoolean()){
            return FunctionList.PORT_CLIMBER_DOWN;
        }
        else if(leftBumper().getAsBoolean()){
            return FunctionList.PORT_CLIMBER_UP;
        }
        else if(start().and(povUp()).getAsBoolean()){
            return FunctionList.ARM_UP;
        }
        else if(start().and(povDown()).getAsBoolean()){
            return FunctionList.ARM_DOWN;
        }
        else if(start().and(povLeft()).getAsBoolean()){
            return FunctionList.REVERSE;
        }
        else if(start().and(povRight()).getAsBoolean()){
            return FunctionList.FEED;
        }
        else {
            return "";
        }
    }
    
}
