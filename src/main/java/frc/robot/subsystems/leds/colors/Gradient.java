package frc.robot.subsystems.leds.colors;

import java.util.ArrayList;

public class Gradient {
    private double h1, s1, v1, h2, s2, v2;
    private int m_range;

    public Gradient(Color color1, Color color2, int from, int to) {
        h1 = color1.getH(); 
        s1 = color1.getS();
        v1 = color1.getV();

        h2 = color2.getH();
        s2 = color2.getS();
        v2 = color2.getV();

        m_range = to - from;
    }

    private double lerp(double num1, double num2, double value) {
        return num1 + (num2 - num1) * value;
    }

    public Color[] genGradient() {
        ArrayList<Color> gradient = new ArrayList<Color>(m_range);

        for (int i = 0; i <= m_range; i++) {
            double value = (1 / m_range) * i;
            
            double h = lerp(h1, h2, value);
            double s = lerp(s1, s2, value);
            double v = lerp(v1, v2, value);
            
            Color currentColor = new Color(h, s, v);

            gradient.add(currentColor);
        }

        Color[] colorArray = new Color[gradient.size()]; // i hate java so much
        return gradient.toArray(colorArray);
    }
}