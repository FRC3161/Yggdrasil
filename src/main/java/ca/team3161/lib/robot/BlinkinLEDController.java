package ca.team3161.lib.robot;

import java.util.concurrent.TimeUnit;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.Spark;

public class BlinkinLEDController extends RepeatingPooledSubsystem{

    Spark blinkinController;
    LEDControllerState state;

    public enum LEDControllerState{
        // From http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
        RED(0.61);

        double PWMValue;

        LEDControllerState(double PWMValue) {
            this.PWMValue = PWMValue;
        }

        double getPWMValue() {
            return this.PWMValue;
        }
    }

    public BlinkinLEDController(int pwmPort) {
        super(100, TimeUnit.SECONDS); // Slower speed, LED values probably won't need to be updated that often
        this.blinkinController = new Spark(pwmPort);

    }

    public void setLEDState(LEDControllerState state) {
        this.state = state;
    }

    @Override
    public void defineResources() {
        require(this.blinkinController);
    }

    @Override
    public void task(){
        this.blinkinController.set(this.state.getPWMValue());
    }

}