package ca.team3161.lib.robot;

import java.util.concurrent.TimeUnit; 

import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.Spark;

/**
 * A class wrapping around a PWM Spark to control a REV Blinkin
 */
public class BlinkinLEDController extends RepeatingPooledSubsystem{

    private final Spark blinkinController;
    private volatile Pattern state;

    public enum Pattern {
        // From http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
        RAINBOW_RAINBOW(-0.99),
        RAINBOW_PARTY(-0.97),
        RAINBOW_OCEAN(-0.95),
        RAINBOW_LAVA(-0.93),
        RAINBOW_FOREST(-0.91),
        RAINBOW_GLITTER(-0.89),
        CONFETTI(-0.87),
        SHOT_RED(-0.85),
        SHOT_BLUE(-0.83),
        SHOT_WHITE(-0.81),
        SINELON_RAINBOW(-0.79),
        SINELON_PARTY(-0.77),
        SINELON_OCEAN(-0.75),
        SINELON_LAVA(-0.73),
        SINELON_FOREST(-0.71),
        BPM_RAINBOW(-0.69),
        BPM_PARTY(-0.67),
        BPM_OCEAN(-0.65),
        BPM_LAVA(-0.63),
        BPM_FOREST(-0.61),
        FIRE_MEDIUM(-0.59),
        FIRE_LARGE(-0.57),
        TWINKLES_RAINBOW(-0.55),
        TWINKLES_PARTY(-0.54),
        TWINKLES_OCEAN(-0.51),
        TWINKLES_LAVA(-0.49),
        TWINKLES_FOREST(-0.47),
        WAVES_RAINBOW(-0.45),
        WAVES_PARTY(-0.43),
        WAVES_OCEAN(-0.41),
        WAVES_LAVA(-0.39),
        WAVES_FOREST(-0.37),
        LARSON_RED(-0.35),
        LARSON_GRAY(-0.33),
        LIGHT_CHASE_RED(-0.31),
        LIGHT_CHASE_BLUE(-0.29),
        LIGHT_CHASE_GRAY(-0.27),
        HEARTBEAT_RED(-0.25),
        HEARTBEAT_BLUE(-0.23),
        HEARTBEAT_WHITE(-0.21),
        HEARTBEAT_GRAY(-0.19),
        BREATH_RED(-0.17),
        BREATH_BLUE(-0.15),
        BREATH_GRAY(-0.13),
        STROBE_RED(-0.11),
        STROBE_BLUE(-0.09),
        STROB_GOLD(-0.07),
        STROBE_WHITE(-0.05),
        E2E_BLEND_TO_BLACK_COLOUR1(-0.03),
        LARSON_SCANNER_COLOUR1(-0.01),
        OFF(0.0), // Assuming OFF is 0, need to test
        LIGHT_CHASE_COLOUR1(0.01),
        HEARTBEAT_SLOW_COLOUR1(0.03),
        HEARTBEAT_MEDIUM_COLOUR1(0.05),
        HEARTBEAT_FAST_COLOUR1(0.07),
        BREATH_SLOW_COLOUR1(0.09),
        BREATH_FAST_COLOUR1(0.11),
        SHOT_COLOUR1(0.13),
        STROBE_COLOUR1(0.15),
        E2E_BLEND_TO_BLACK_COLOUR2(0.17),
        LARSON_SCANNER_COLOUR2(0.19),
        LIGHT_CHASE_COLOUR2(0.21),
        HEARTBEAT_SLOW_COLOUR2(0.23),
        HEARTBEAT_MEDIUM_COLOUR2(0.25),
        HEARTBEAT_FAST_COLOUR2(0.27),
        BREATH_SLOW_COLOUR2(0.29),
        BREATH_FAST_COLOUR2(0.31),
        SHOT_COLOUR2(0.33),
        STROBE_COLOUR2(0.35),
        SPARKLE_COLOUR1_ON_2(0.37),
        SPARKLE_COLOUR2_ON_1(0.39),
        GRADIENT_COLOUR1_AND_2(0.41),
        BPM_COLOUR1_AND_2(0.43),
        E2E_BLEND_COLOUR1_TO_2(0.47),
        E2E_BLEND_COLOUR2_TO_1(0.47), // Doesn't state that it's 2 to 1 in the pdf but it's the only possible option other than 1 to 2
        COLOUR1_AND_2(0.49),
        TWINKLES_COLOUR1_AND_2(0.51),
        WAVES_COLOUR1_AND_2(0.53),
        SINELON_COLOUR1_AND_2(0.55),
        SOLID_HOT_PINK(0.57),
        SOLID_DARK_RED(0.59),
        SOLID_RED(0.61),
        SOLID_RED_ORANGE(0.63),
        SOLID_ORANGE(0.65),
        SOLID_GOLD(0.67),
        SOLID_YELLOW(0.69),
        SOLID_LAWN_GREEN(0.71),
        SOLID_LIME(0.73),
        SOLID_DARK_GREEN(0.75),
        SOLID_GREEN(0.77),
        SOLID_BLUE_GREEN(0.79),
        SOLID_AQUA(0.81),
        SOLID_SKY_BLUE(0.83),
        SOLID_DARK_BLUE(0.85),
        SOLID_BLUE(0.87),
        SOLID_BLUE_VIOLET(0.89),
        SOLID_VIOLET(0.91),
        SOLID_WHITE(0.93),
        SOLID_GRAY(0.95),
        SOLID_DARK_GRAY(0.97),
        SOLID_BLACK(0.99);

        double PWMValue;

        Pattern(double PWMValue) {
            this.PWMValue = PWMValue;
        }

        double getPWMValue() {
            return this.PWMValue;
        }
    }

    /**
     * @param pwmPort   The PWM port of the Blinkin
     */
    public BlinkinLEDController(int pwmPort) {
        super(100, TimeUnit.SECONDS); // Slower speed, LED values probably won't need to be updated that often
        this.blinkinController = new Spark(pwmPort);

    }

    /**
    * Sets the Blinkin LED Controller to the given pattern
    * @param  pattern   The pattern you want to set the controller to 
    */
    public void setLEDState(Pattern pattern) {
        this.state = pattern;
    }

    @Override
    public void defineResources() {
        require(this.blinkinController);
    }

    @Override
    public void task() {
        this.blinkinController.set(this.state.getPWMValue());
    }

}