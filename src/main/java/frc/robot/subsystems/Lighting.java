package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightingConstants;

public class Lighting extends SubsystemBase {
    private final int LEDS_PER_ANIMATION = 5;
    private final CANdle m_candle = new CANdle(LightingConstants.CANDLE_CANID);
    private int m_candleChannel = 0;
    private boolean m_clearAllAnims = false;
    private boolean m_last5V = false;
    private boolean m_animDirection = false;
    private boolean m_setAnim = false;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
        Empty
    }
    private AnimationTypes m_currentAnimation;

    public Lighting() {
        changeAnimation(AnimationTypes.Rainbow);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.On;
        m_candle.configAllSettings(configAll, 100);
    }

    public void toggle5VOverride() {
        System.out.println("State is: " + m_last5V);
        m_candle.configV5Enabled(m_last5V);
        m_last5V = !m_last5V;
    }
    public void toggleAnimDirection() {
        m_animDirection = !m_animDirection;
    }
    public int getMaximumAnimationCount() {
        return m_candle.getMaxSimultaneousAnimationCount();
    }

    public void incrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Empty); break;
            case Empty: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Empty); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case Empty: changeAnimation(AnimationTypes.TwinkleOff); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    public void setAnim(AnimationTypes anim) {
        changeAnimation(anim);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            default:
            case ColorFlow:
                m_candleChannel = 0;
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDS_PER_ANIMATION, Direction.Forward, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Fire:
                m_candleChannel = 1;
                m_toAnimate = new FireAnimation(0.5, 0.7, LEDS_PER_ANIMATION, 0.8, 0.5, m_animDirection, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Larson:
                m_candleChannel = 2;
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 0.1, LEDS_PER_ANIMATION, BounceMode.Front, 3, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Rainbow:
                m_candleChannel = 3;
                m_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case RgbFade:
                m_candleChannel = 4;
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_PER_ANIMATION, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case SingleFade:
                m_candleChannel = 5;
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_PER_ANIMATION, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Strobe:
                m_candleChannel = 6;
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.01, LEDS_PER_ANIMATION, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Twinkle:
                m_candleChannel = 7;
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LEDS_PER_ANIMATION, TwinklePercent.Percent42, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case TwinkleOff:
                m_candleChannel = 8;
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_PER_ANIMATION, TwinkleOffPercent.Percent76, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Empty:
                m_candleChannel = 9;
                m_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;

            case SetAll:
                m_toAnimate = null;
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }

    public void clearAllAnims() {m_clearAllAnims = true;}

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
