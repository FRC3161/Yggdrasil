package ca.team3161.lib.utils;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiConsumer;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DualSmartDashboardTuner extends RepeatingPooledSubsystem implements LifecycleListener {

    protected final Preferences prefs;
    protected final String labelA;
    protected final String labelB;
    protected final double defaultValueA;
    protected final double defaultValueB;
    protected final BiConsumer<Double, Double> consumer;
    protected final ReentrantLock writeLock = new ReentrantLock();
    protected volatile double lastValueA;
    protected volatile double lastValueB;
    protected volatile boolean continuous;

    public DualSmartDashboardTuner(int period, String labelA, String labelB, double defaultValueA, double defaultValueB,
            BiConsumer<Double, Double> consumer) {
        super(period, TimeUnit.MILLISECONDS);
        this.prefs = Preferences.getInstance();
        this.labelA = labelA;
        this.labelB = labelB;
        this.consumer = consumer;

        this.defaultValueA = prefs.getDouble(labelA, defaultValueA);
        this.defaultValueB = prefs.getDouble(labelB, defaultValueB);

        SmartDashboard.putNumber(labelA, this.defaultValueA);
        SmartDashboard.putNumber(labelB, this.defaultValueB);
    }

    public DualSmartDashboardTuner(String labelA, String labelB, double defaultValueA, double defaultValueB,
            BiConsumer<Double, Double> consumer) {
        this(500, labelA, labelB, defaultValueA, defaultValueB, consumer);
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
        double valA = SmartDashboard.getNumber(labelA, defaultValueA);
        double valB = SmartDashboard.getNumber(labelB, defaultValueB);
        consumer.accept(valA, valB);

        double currentValueA = SmartDashboard.getNumber(labelA, this.defaultValueA);
        double currentValueB = SmartDashboard.getNumber(labelB, this.defaultValueB);
        boolean changedA = currentValueA != lastValueA;
        boolean changedB = currentValueB != lastValueB;
        boolean changed = changedA || changedB;

        if (!changed && !continuous) {
            return;
        }
        try {
            writeLock.lockInterruptibly();
            consumer.accept(currentValueA, currentValueB);
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
