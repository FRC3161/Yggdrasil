package ca.team3161.lib.utils;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardTuner extends RepeatingPooledSubsystem implements LifecycleListener {

    protected final Preferences prefs;
    protected final String label;
    protected final double defaultValue;
    protected final Consumer<Double> consumer;
    protected final ReentrantLock writeLock = new ReentrantLock();
    protected volatile double lastValue;
    protected volatile boolean continuous;

    public SmartDashboardTuner(String label, double defaultValue, Consumer<Double> consumer) {
        this(500, label, defaultValue, consumer);
    }

    public SmartDashboardTuner(int period, String label, double defaultValue, Consumer<Double> consumer) {
        super(period, TimeUnit.MILLISECONDS);
        this.prefs = Preferences.getInstance();
        this.label = label;
        this.consumer = consumer;

        this.defaultValue = prefs.getDouble(label, defaultValue);
        this.lastValue = defaultValue;

        SmartDashboard.putNumber(label, this.defaultValue);
    }

    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    public boolean isContinuous() {
        return this.continuous;
    }

    @Override
    public void defineResources() { }

    @Override
    public void task() {
        double currentValue = SmartDashboard.getNumber(label, this.defaultValue);
        boolean changed = currentValue != lastValue;
        if (!changed && !continuous) {
            return;
        }
        try {
            writeLock.lockInterruptibly();
            consumer.accept(currentValue);
        } catch (InterruptedException e) {
        } finally {
            writeLock.unlock();
        }
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
