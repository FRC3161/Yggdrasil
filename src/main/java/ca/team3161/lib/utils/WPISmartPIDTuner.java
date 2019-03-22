package ca.team3161.lib.utils;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.PIDController;

public class WPISmartPIDTuner extends RepeatingPooledSubsystem implements LifecycleListener {

    protected final String prefix;
    protected final PIDController pid;
    protected final SmartDashboardTuner pTuner;
    protected final SmartDashboardTuner iTuner;
    protected final SmartDashboardTuner dTuner;

    public WPISmartPIDTuner(String prefix, PIDController pid, double defaultKP, double defaultKI, double defaultKD) {
        super(500, TimeUnit.MILLISECONDS);
        this.prefix = prefix;
        this.pid = pid;

        this.pTuner = new SmartDashboardTuner(getKPLabel(), defaultKP, pid::setP);
        this.iTuner = new SmartDashboardTuner(getKILabel(), defaultKI, pid::setI);
        this.dTuner = new SmartDashboardTuner(getKDLabel(), defaultKD, pid::setD);
    }

    protected String getKPLabel() {
        return prefix + "-kP";
    }

    protected String getKILabel() {
        return prefix + "-kI";
    }

    protected String getKDLabel() {
        return prefix + "-kD";
    }

    @Override
    public void defineResources() {
        require(pid);
    }

    @Override
    public void task() { }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        this.pTuner.lifecycleStatusChanged(previous, current);
        this.iTuner.lifecycleStatusChanged(previous, current);
        this.dTuner.lifecycleStatusChanged(previous, current);
        switch (current) {
            case NONE:
            case ON_INIT:
            case ON_AUTO:
                cancel();
                break;
            case ON_DISABLED:
            case ON_TELEOP:
            case ON_TEST:
                start();
                break;
            default:
                throw new IllegalStateException(current.toString());
        }
    }

}
