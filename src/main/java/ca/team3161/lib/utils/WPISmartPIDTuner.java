package ca.team3161.lib.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.PIDController;

public class WPISmartPIDTuner extends RepeatingPooledSubsystem implements LifecycleListener {

    protected final PIDController pid;
    protected final List<LifecycleListener> tuners;

    protected WPISmartPIDTuner(PIDController pid, List<LifecycleListener> tuners) {
        super(500, TimeUnit.MILLISECONDS);
        this.pid = pid;
        this.tuners = tuners;
    }

    protected void addTuner(SmartDashboardTuner tuner) {
        this.tuners.add(tuner);
    }

    @Override
    public void defineResources() {
        require(pid);
    }

    @Override
    public void task() { }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        tuners.forEach(t -> t.lifecycleStatusChanged(previous, current));
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

    public static class Builder {
        private final List<Function<String, Function<PIDController, LifecycleListener>>> generators = new ArrayList<>();

        private void addTuner(String label, double defaultValue, Function<PIDController, Consumer<Double>> consumer) {
            generators.add(
                prefix -> controller -> new SmartDashboardTuner(prefix + "-" + label, defaultValue, consumer.apply(controller))
            );
        }

        private void addDualTuner(String labelA, double defaultValueA, String labelB, double defaultValueB, Function<PIDController, BiConsumer<Double, Double>> consumer) {
            generators.add(
                    prefix ->
                        controller -> new DualSmartDashboardTuner(prefix + "-" + labelA, prefix + "-" + labelB,
                            defaultValueA, defaultValueB, consumer.apply(controller))
            );
        }

        public Builder kP(double kP) {
            addTuner("kP", kP, controller -> d -> controller.setP(d));
            return this;
        }

        public Builder kI(double kI) {
            addTuner("kI", kI, controller -> d -> controller.setI(d));
            return this;
        }

        public Builder kD(double kD) {
            addTuner("kD", kD, controller -> d -> controller.setD(d));
            return this;
        }

        public Builder kF(double kF) {
            addTuner("kF", kF, controller -> d -> controller.setF(d));
            return this;
        }

        public Builder absoluteTolerance(double tol) {
            addTuner("absTolerance", tol, controller -> d -> controller.setAbsoluteTolerance(d));
            return this;
        }

        public Builder percentTolerance(double tol) {
            addTuner("pctTolerance", tol, controller -> d -> controller.setPercentTolerance(d));
            return this;
        }

        public Builder outputRange(double min, double max) {
            addDualTuner("minOutput", min, "maxOutput", max, controller -> (d1, d2) -> controller.setOutputRange(d1, d2));
            return this;
        }

        public Builder inputRange(double min, double max) {
            addDualTuner("minInput", min, "maxInput", max, controller -> (d1, d2) -> controller.setInputRange(d1, d2));
            return this;
        }

        public WPISmartPIDTuner build(PIDController controller) {
            final String name = controller.getName();
            final String prefix;
            if (name != null && !name.isEmpty()) {
                prefix = name;
            } else {
                prefix = "WPISmartPIDTuner";
            }
            List<LifecycleListener> tuners
                = generators
                    .stream()
                    .map(g -> g.apply(prefix))
                    .map(f -> f.apply(controller))
                    .collect(Collectors.toList());
            return new WPISmartPIDTuner(controller, tuners);
        }
    }

}
