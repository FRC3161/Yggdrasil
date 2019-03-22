package ca.team3161.lib.utils;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WPISmartPIDTuner extends RepeatingPooledSubsystem implements LifecycleListener {

    protected final Preferences prefs;

    protected final String prefix;
    protected final PIDController pid;
    protected final double defaultKP;
    protected final double defaultKI;
    protected final double defaultKD;

    public WPISmartPIDTuner(String prefix, PIDController pid, double defaultKP, double defaultKI, double defaultKD) {
        super(500, TimeUnit.MILLISECONDS);
        this.prefix = prefix;
        this.pid = pid;
        this.prefs = Preferences.getInstance();

        this.defaultKP = prefs.getDouble(getKPLabel(), defaultKP);
        this.defaultKI = prefs.getDouble(getKILabel(), defaultKI);
        this.defaultKD = prefs.getDouble(getKDLabel(), defaultKD);

        SmartDashboard.putNumber(getKPLabel(), this.defaultKP);
        SmartDashboard.putNumber(getKILabel(), this.defaultKI);
        SmartDashboard.putNumber(getKDLabel(), this.defaultKD);
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
    public void task() throws Exception {
        pid.setP((float) SmartDashboard.getNumber(getKPLabel(), defaultKP));
        pid.setI((float) SmartDashboard.getNumber(getKILabel(), defaultKI));
        pid.setD((float) SmartDashboard.getNumber(getKDLabel(), defaultKD));
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
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
