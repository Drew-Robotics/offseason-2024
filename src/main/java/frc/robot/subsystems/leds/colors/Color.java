package frc.robot.subsystems.leds.colors;

import java.util.function.IntSupplier;

public class Color {
    private IntSupplier m_r, m_g, m_b;
    
    public Color(IntSupplier rSup, IntSupplier gSup, IntSupplier bSup) {
        m_r = rSup;
        m_g = gSup;
        m_b = bSup;
    }

    public Color(int r, int g, int b) {
        m_r = () -> r;
        m_g = () -> g;
        m_b = () -> b;
    }

    public int getR() { return m_r.getAsInt(); }
    public int getG() { return m_g.getAsInt(); }
    public int getB() { return m_b.getAsInt(); }
}
