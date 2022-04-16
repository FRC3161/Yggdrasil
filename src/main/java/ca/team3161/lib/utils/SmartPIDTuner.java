package ca.team3161.lib.utils;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.pid.PID;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

public class SmartPIDTuner extends RepeatingPooledSubsystem implements LifecycleListener {

    protected final String prefix;
    protected final PID<?, ?> pid;
    protected final SmartDashboardTuner pTuner;
    protected final SmartDashboardTuner iTuner;
    protected final SmartDashboardTuner dTuner;

    public SmartPIDTuner(String prefix, PID<?, ?> pid, double defaultKP, double defaultKI, double defaultKD) {
        super(500, TimeUnit.MILLISECONDS);
        this.prefix = prefix;
        this.pid = pid;

        this.pTuner = new SmartDashboardTuner(getKPLabel(), defaultKP, v -> pid.setkP(v.floatValue()));
        this.iTuner = new SmartDashboardTuner(getKILabel(), defaultKI, v -> pid.setkI(v.floatValue()));
        this.dTuner = new SmartDashboardTuner(getKDLabel(), defaultKD, v -> pid.setkD(v.floatValue()));
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
    public void lifecycleStatusChanged(LifecycleEvent event) {
        this.pTuner.lifecycleStatusChanged(event);
        this.iTuner.lifecycleStatusChanged(event);
        this.dTuner.lifecycleStatusChanged(event);
        switch (event.getMode()) {
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
                throw new IllegalStateException(event.getMode().toString());
        }
    }

}
