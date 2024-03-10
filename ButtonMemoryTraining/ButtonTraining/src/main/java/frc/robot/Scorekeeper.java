package frc.robot;

public class Scorekeeper {
    private int score = 0;
    public Scorekeeper() {}

    public int getScore() {
        return score;
    }

    public void add() {
        score++;
    }
    
    public void reset() {
        score = 0;
    }
}
