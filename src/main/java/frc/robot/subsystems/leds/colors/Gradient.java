package frc.robot.subsystems.leds.colors;

import java.util.ArrayList;

public class Gradient {
    private int r1, g1, b1, r2, g2, b2;
    private int m_range;

    public Gradient(Color color1, Color color2, int from, int to) {
        r1 = color1.getR(); 
        g1 = color1.getG();
        b1 = color1.getB();

        r2 = color2.getR();
        g2 = color2.getG();
        b2 = color2.getB();

        m_range = to - from;
    }

    private Double lerp(int num1, int num2, double value) {
        double num1_double = num1;
        double num2_double = num2;
        
        return num1_double + (num2_double - num1_double) * value;
    }

    public Color[] genGradient() {
        ArrayList<Color> gradient = new ArrayList<Color>(m_range);

        for (int i = 0; i <= m_range; i++) {
            double value = (1 / m_range) * i;
            
            int r = (int) Math.round(lerp(r1, r2, value) * 255);
            int g = (int) Math.round(lerp(g1, g2, value) * 255);
            int b = (int) Math.round(lerp(b1, b2, value) * 255);
            
            Color currentColor = new Color(r, g, b);

            gradient.add(currentColor);
        }

        Color[] colorArray = new Color[gradient.size()]; // i hate java so much
        return gradient.toArray(colorArray);
    }
}
