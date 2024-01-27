package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {

    public static Spark blinkin;

    public Blinkin(){
        blinkin = new Spark(5);
}

    public void red(){
        blinkin.set(0.61);
    }

    public void green(){
        blinkin.set(0.73);
    }

    public void blue(){
        blinkin.set(0.87);
    }

    public void party(){
        blinkin.set(-.97);
    }

    public void greenWaves(){
        blinkin.set(-.37);
    }
    
}
