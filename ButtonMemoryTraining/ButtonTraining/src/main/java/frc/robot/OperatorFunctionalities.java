package frc.robot;

import frc.robot.Constants.FunctionList;

public final class OperatorFunctionalities {

    public static String getFunction() {
        String result = "";
        switch (Randomizer.getRandomNumber(0, 112)) {
            case 0:
                result = FunctionList.AMP;
            case 1:
                result = FunctionList.SHOOT_DISTANCE;
            case 2:
                result = FunctionList.INTAKE;
            case 3:
                result = FunctionList.SHOOT_SUBWOOFER;
            case 4:
                result = FunctionList.STARBOARD_CLIMBER_UP;
            case 5:
                result = FunctionList.STARBOARD_CLIMBER_DOWN;
            case 6:
                result = FunctionList.OVERRIDE;
            case 7:
                result = FunctionList.HOME_CLIMBERS;
            case 8:
                result = FunctionList.MANOEUVER_DRIVE;
            case 9:
                result = FunctionList.ARM_UP;
            case 10:
                result = FunctionList.FEED;
            case 11:
                result = FunctionList.ARM_DOWN;
            case 12:
                result = FunctionList.REVERSE;
        }
        return result;
    }

}