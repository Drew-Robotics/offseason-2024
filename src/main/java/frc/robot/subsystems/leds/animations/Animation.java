package frc.robot.subsystems.leds.animations;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.ctre.phoenix.led.CANdle;

import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.leds.colors.Color;

public class Animation {
    
    protected CANdle m_candle;
    protected Color m_blank = new Color(0, 0, 0, 0);
    /**
     * Using custom Array instead of changing each LED individualy
     */
    protected ArrayList<ArrayList<Color>> m_LEDStrip = 
        new ArrayList<ArrayList<Color>>(LEDConstants.kNumOfLEDs);

    protected Animation(CANdle candle) {
        m_candle = candle;

        for (int i = 0; i <= LEDConstants.kNumOfLEDs; i++) {
            m_LEDStrip.get(i).add(m_blank);
        }
    }

    protected Color avarageColors(Color color1, Color color2) {
        double h = (color1.getH() + color2.getH()) / 2;
        double s = (color1.getS() + color2.getS()) / 2;
        double v = (color1.getV() + color2.getV()) / 2;

        return new Color(h, s ,v);
    }

    protected void setLEDs(Color color, int from, int to) {
        for (int i = from; i <= to; i ++) 
            m_LEDStrip.get(i).add(color);
    }

    protected void setLED(Color color, int led) {
        m_LEDStrip.get(led).add(color);
    }

    /**
     * 
     * @param color color of leds
     * @param from starting led
     * @param to last led
     * @param gradientWidth the width of the gradient on both sides
     */
    protected void setLEDsGradient(Color color, int from, int to, int gradientWidth) {
        int solidWidth = (to - from) - (gradientWidth * 2);
        if (solidWidth < 0)
            solidWidth = 0;

        

    }
}