package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Dashboard {
    public Dashboard(Test test) {
        Shuffleboard.getTab("test").addString("function", ()-> test.getTask());
        Shuffleboard.getTab("test").addInteger("score", ()-> test.getScore());
        Shuffleboard.getTab("test").addBoolean("correct", ()-> test.isCorrect());
    }
}
