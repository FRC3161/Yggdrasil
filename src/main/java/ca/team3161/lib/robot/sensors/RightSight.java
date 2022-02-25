package ca.team3161.lib.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class RightSight extends DigitalInput {

    protected volatile boolean inverted;

    public RightSight(int channel) {
        super(channel);
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public boolean get() {
        boolean val = super.get();
        if (inverted) {
            return !val;
        } else {
            return val;
        }
    }

}
