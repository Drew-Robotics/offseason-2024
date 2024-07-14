package frc.robot.subsystems.leds.colors;

public class HSV {
    private double m_h, m_s, m_v;

    public HSV(double h, double s, double v) {
        m_h = h;
        m_s = s;
        m_v = v;
    }

    /**
     * Converts RGB to HSV
     * 
     * @param r red 
     * @param g blue
     * @param b green
     */ 
    public HSV(int r, int g, int b) {
        double hue, saturation, brightness;
        int cmax = (r > g) ? r : g;
        if (b > cmax) cmax = b;
        int cmin = (r < g) ? r : g;
        if (b < cmin) cmin = b;
        brightness = ((double) cmax) / 255.0f;
        if (cmax != 0)
            saturation = ((double) (cmax - cmin)) / ((double) cmax);
        else
            saturation = 0;
        if (saturation == 0)
            hue = 0;
        else {
            double redc = ((double) (cmax - r)) / ((double) (cmax - cmin));
            double greenc = ((double) (cmax - g)) / ((double) (cmax - cmin));
            double bluec = ((double) (cmax - b)) / ((double) (cmax - cmin));
            if (r == cmax)
                hue = bluec - greenc;
            else if (g == cmax)
                hue = 2.0f + redc - bluec;
            else
                hue = 4.0f + greenc - redc;
            hue = hue / 6.0f;
            if (hue < 0)
                hue = hue + 1.0f;
        }

        m_h = hue;
        m_s = saturation;
        m_v = brightness;
    }

    public double getH() { return m_h; };
    public double getS() { return m_s; };
    public double getV() { return m_v; };
}