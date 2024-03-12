// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class Test {
  private final StringXboxController controller;
  private final Scorekeeper scorekeeper;
  private String current;
  private boolean correct;
  /** Creates a new Test. */
  public Test(StringXboxController controller, Scorekeeper scorekeeper) {
    this.controller = controller;
    this.scorekeeper = scorekeeper;
    current = "";
    correct = false;
  }

  public Command activate() {
    correct = false;
    return new ParallelRaceGroup(
      new WaitCommand(3),
      new InstantCommand(() -> run())
    );
  }

  public void run() {
    current = OperatorFunctionalities.getFunction();
    if(current == controller.getActiveFunction()) {
      scorekeeper.add();
      correct = true;
      System.out.println("correct");
    }
    else {
      correct = false;
      System.out.println("incorrect");
    }
  }

  public String getTask() {
    return current;
  }

  public int getScore() {
    return scorekeeper.getScore();
  }

  public boolean isCorrect() {
    return correct;
  }
}
