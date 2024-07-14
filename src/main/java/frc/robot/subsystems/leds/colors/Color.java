package frc.robot.subsystems.leds.colors;

public class Color {
    private int m_r, m_g, m_b;
    private HSV m_HSV;

    public Color(int r, int g, int b) {
        m_r = r;
        m_g = g;
        m_b = b;

        m_HSV = new HSV(r, g, b);
    }

    /**
     * Converts HSV to RGB
     * 
     * @param h Hue
     * @param s Saturation
     * @param v Value
     * 
     * @return Color with RGB
     */
    public Color(double h, double s, double v) {
        int r = 0, g = 0, b = 0;
        if (s == 0) {
            r = g = b = (int) (v * 255.0f + 0.5f);
        } else {
            double h2 = (h - (double)Math.floor(h)) * 6.0f;
            double f = h - (double)java.lang.Math.floor(h);
            double p = v * (1.0f - s);
            double q = v * (1.0f - s * f);
            double t = v * (1.0f - (s * (1.0f - f)));
            switch ((int) h2) {
            case 0:
                r = (int) (v * 255.0f + 0.5f);
                g = (int) (t * 255.0f + 0.5f);
                b = (int) (p * 255.0f + 0.5f);
                break;
            case 1:
                r = (int) (q * 255.0f + 0.5f);
                g = (int) (v * 255.0f + 0.5f);
                b = (int) (p * 255.0f + 0.5f);
                break;
            case 2:
                r = (int) (p * 255.0f + 0.5f);
                g = (int) (v * 255.0f + 0.5f);
                b = (int) (t * 255.0f + 0.5f);
                break;
            case 3:
                r = (int) (p * 255.0f + 0.5f);
                g = (int) (q * 255.0f + 0.5f);
                b = (int) (v * 255.0f + 0.5f);
                break;
            case 4:
                r = (int) (t * 255.0f + 0.5f);
                g = (int) (p * 255.0f + 0.5f);
                b = (int) (v * 255.0f + 0.5f);
                break;
            case 5:
                r = (int) (v * 255.0f + 0.5f);
                g = (int) (p * 255.0f + 0.5f);
                b = (int) (q * 255.0f + 0.5f);
                break;
            }
        }
        
        m_r = r;
        m_g = g;
        m_b = b;
    }

    public int getR() { return m_r; }
    public int getG() { return m_g; }
    public int getB() { return m_b; }

    public double getH() { return m_HSV.getH(); };
    public double getS() { return m_HSV.getS(); };
    public double getV() { return m_HSV.getV(); };
}