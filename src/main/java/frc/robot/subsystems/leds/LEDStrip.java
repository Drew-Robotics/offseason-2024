package frc.robot.subsystems.leds;

import java.util.ArrayList;

import com.ctre.phoenix.led.CANdle;

import frc.robot.subsystems.leds.colors.Color;
import frc.robot.subsystems.leds.colors.Gradient;

public class LEDStrip {
    protected CANdle m_candle;
    protected LED m_blank = new LED(new Color(0, 0, 0), 0);
    /**
     * Using custom Array instead of changing each LED individualy
     */
    protected ArrayList<ArrayList<LED>> m_LEDStrip = new ArrayList<ArrayList<LED>>();

    /**
     * Gets the LED at an index with the highest priority
     * @param LED Index for led wanted
     */
    protected LED getLED(int index) {
        int max = 0;
        ArrayList<LED> maxes = new ArrayList<LED>();

        for (LED i : m_LEDStrip.get(index))
            if (i.getPriority() > max)
                max = i.getPriority();
        
        for (LED j : m_LEDStrip.get(index))
            if (j.getPriority() == max) {
                maxes.add(j);
            };
        
        return maxes.get(0);
    }

    protected Color avarageColors(Color color1, Color color2) {
        double h = (color1.getH() + color2.getH()) / 2;
        double s = (color1.getS() + color2.getS()) / 2;
        double v = (color1.getV() + color2.getV()) / 2;

        return new Color(h, s ,v);
    }

    protected void setLEDs(LED led, int from, int to) {
        for (int i = from; i <= to; i ++) 
            m_LEDStrip.get(i).add(led);
    }

    protected void setLED(LED led, int index) {
        m_LEDStrip.get(index).add(led);
    }

    protected void setGradient(Color Color1, Color Color2, int from, int to, int priority) {
        Gradient gradient = new Gradient(Color1, Color2, from, to);
        Color[] gradientColors = gradient.genGradient();

        int j = 0;
        for (int i = from; i <= to; i++) {
            setLED(new LED(gradientColors[j], priority), i);
            j++;
        }
    }
}
