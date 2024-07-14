package frc.robot.subsystems.leds;

import frc.robot.subsystems.leds.colors.Color;

public class LED {
    private int m_priority;
    private Color m_color;

    public LED(Color color, int priority) {
        m_priority = priority;
        m_color = color;
    }

    public LED(Color color) {
        m_priority = 1;
        m_color = color;
    }

    public int getPriority() { return m_priority; };
    public Color getColor() { return m_color; };
}
