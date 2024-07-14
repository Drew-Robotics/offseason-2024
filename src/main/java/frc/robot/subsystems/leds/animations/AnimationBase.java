package frc.robot.subsystems.leds.animations;

import com.ctre.phoenix.led.CANdle;

import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LEDStrip;
import frc.robot.subsystems.leds.colors.Color;

public class AnimationBase extends LEDStrip {
    protected AnimationBase(CANdle candle) {
        m_candle = candle;

        for (int i = 0; i <= LEDConstants.kNumOfLEDs; i++) {
            m_LEDStrip.get(i).add(m_blank);
        }
    }

    /**
     * @param color color of leds
     * @param led led that is at the center of the pattern
     * @param width the width of the pattern
     * @param gradientWidth the width of the gradient on both sides within the pattern
     */
//     protected void setPointFadeInOut(Color color, int led, int width, int gradientWidth, int priority) {
//         int solidWidth = width - (gradientWidth * 2);
//         if (solidWidth < 0) {
//             solidWidth = 0;
//         } else {
//             setLEDs(
//                 new LED(color, priority), 
//                 led - (((int) Math.floor(solidWidth / 2)) - 1), 
//                 led + (((int) Math.floor(solidWidth / 2)) - 1)
//             );
//         };

//         int halfWidth = ((int) Math.floor(width / 2)) - 1;
//         if (halfWidth < 0)
//             halfWidth = 0;
//     }
}