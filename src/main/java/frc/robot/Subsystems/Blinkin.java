package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;


public class Blinkin {

    public static Spark blinkin;

    private static Blinkin m_Instance = null;


    public Blinkin() {
        blinkin = new Spark(Constants.Blinkin.blinkinPort);

    }

    // Blinkin color/pattern values -
    // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf


    public void rainbowRGB() {
        blinkin.set(-.99);
    }

    public void rainbowParty() {
        blinkin.set(-.97);
    }

    public void rainbowOcean() {
        blinkin.set(-.95);
    }

    public void rainbowLava() {
        blinkin.set(-.93);
    }

    public void rainbowForest() {
        blinkin.set(-.91);
    }

    public void rainbowGlitter() {
        blinkin.set(-.89);
    }

    public void confetti() {
        blinkin.set(-.87);
    }

    public void redShot() {
        blinkin.set(-.85);
    }

    public void blueShot() {
        blinkin.set(-.83);
    }

    public void whiteShot() {
        blinkin.set(-.81);
    }

    public void sinelonRGB() {
        blinkin.set(-.79);
    }

    public void sinelonParty() {
        blinkin.set(-.77);
    }

    public void sinelonOcean() {
        blinkin.set(-.75);
    }

    public void sinelonLava() {
        blinkin.set(-.73);
    }

    public void sinelonForest() {
        blinkin.set(-.71);
    }

    public void BPMRGB() {
        blinkin.set(-.69);
    }

    public void BPMParty() {
        blinkin.set(-.67);
    }

    public void BPMOcean() {
        blinkin.set(-.65);
    }

    public void BPMLava() {
        blinkin.set(-.63);
    }

    public void BPMForest() {
        blinkin.set(-.61);
    }

    public void fireMedium() {
        blinkin.set(-.59);
    }

    public void fireLarge() {
        blinkin.set(-.57);
    }

    public void twinklesRGB() {
        blinkin.set(-.55);
    }

    public void twinklesParty() {
        blinkin.set(-.53);
    }

    public void twinklesOcean() {
        blinkin.set(-.51);
    }

    public void twinklesLava() {
        blinkin.set(-.49);
    }

    public void twinklesForest() {
        blinkin.set(-.47);
    }

    public void wavesRGB() {
        blinkin.set(-.45);
    }

    public void wavesParty() {
        blinkin.set(-.43);
    }

    public void wavesOcean() {
        blinkin.set(-.41);
    }

    public void wavesLava() {
        blinkin.set(-.39);
    }

    public void wavesForest() {
        blinkin.set(-.37);
    }

    public void scannerRed() {
        blinkin.set(-.35);
    }

    public void scannerGray() {
        blinkin.set(-.33);
    }

    public void chaseRed() {
        blinkin.set(-.31);
    }

    public void chaseBlue() {
        blinkin.set(-.29);
    }

    public void chaseGray() {
        blinkin.set(-.27);
    }

    public void heartbeatRed() {
        blinkin.set(-.25);
    }

    public void heartbeatBlue() {
        blinkin.set(-.23);
    }

    public void heartbeatGray() {
        blinkin.set(-.21);
    }

    public void breathRed() {
        blinkin.set(-.19);
    }

    public void breathBlue() {
        blinkin.set(-.17);
    }

    public void breathGray() {
        blinkin.set(-.15);
    }

    public void solidblue() {
        blinkin.set(.87);
    }

    public void lightchase() {
        blinkin.set(.21);
    }

    public void Heartbeatslow() {
        blinkin.set(.03);
    }




    public void heartbeatmedium() {
    blinkin.set(.03);
    }

    public void heartbeatfast() {
    blinkin.set(.05);
    }

    public void breathslow() {
    blinkin.set(.07);
    }

    public void breathfast() {
    blinkin.set(.11);
    }

    public void shot() {
    blinkin.set(.13);
    }


    public void blendtoblack() {
    blinkin.set(.17);
    }

    public void larsonscanner() {
    blinkin.set(.19);
    }

    public void hotpink() {
    blinkin.set(.57);
    }

    public void normsparkle() {
    blinkin.set(.37);
    }

    public void crazysparkle() {
    blinkin.set(.39);
    }

    public void crazyBPM() {
    blinkin.set(.41);
    }

    public void crazyblend() {
    blinkin.set(.45);
    }

    public void blend() {
    blinkin.set(.47);
    }

    public void setuppattern() {
    blinkin.set(.49);
    }

    public void crazytwinkle() {
    blinkin.set(.51);
    }

    public void colorwave() {
    blinkin.set(.53);
    }

    public void sinelon() {
    blinkin.set(.55);
    }

    public void green() {
        blinkin.set(.71);
    }

    public void lime() {
        blinkin.set(.73);
    }

    public void aqua(){
        blinkin.set(.81);
    }

    public void orange(){
        blinkin.set(.65);
    }



    // public void BPMLava() {
    // blinkin.set(-.63);
    // }

    // public void BPMForest() {
    // blinkin.set(-.61);
    // }

    // public void fireMedium() {
    // blinkin.set(-.59);
    // }

    // public void fireLarge() {
    // blinkin.set(-.57);
    // }

    // public void twinklesRGB() {
    // blinkin.set(-.55);
    // }

    // public void twinklesParty() {
    // blinkin.set(-.53);
    // }

    // public void twinklesOcean() {
    // blinkin.set(-.51);
    // }

    // public void twinklesLava() {
    // blinkin.set(-.49);
    // }

    // public void twinklesForest() {
    // blinkin.set(-.47);
    // }

    // public void wavesRGB() {
    // blinkin.set(-.45);
    // }

    // public void wavesParty() {
    // blinkin.set(-.43);
    // }

    // public void wavesOcean() {
    // blinkin.set(-.41);
    // }

    // public void wavesLava() {
    // blinkin.set(-.39);
    // }

    // public void wavesForest() {
    // blinkin.set(-.37);
    // }

    // public void scannerRed() {
    // blinkin.set(-.35);
    // }

    // public void scannerGray() {
    // blinkin.set(-.33);
    // }

    // public void chaseRed() {
    // blinkin.set(-.31);
    // }

    // public void chaseBlue() {
    // blinkin.set(-.29);
    // }

    // public void chaseGray() {
    // blinkin.set(-.27);
    // }

    // public void heartbeatRed() {
    // blinkin.set(-.25);
    // }

    // public void heartbeatBlue() {
    // blinkin.set(-.23);
    // }

    // public void heartbeatGray() {
    // blinkin.set(-.21);
    // }

    // public void breathRed() {
    // blinkin.set(-.19);
    // }

    // public void breathBlue() {
    // blinkin.set(-.17);
    // }

    // public void breathGray() {
    // blinkin.set(-.15);
    // }

    public static Blinkin getInstance() {
        if (m_Instance == null) {
            m_Instance = new Blinkin();
        }

        return m_Instance;
    }

}
