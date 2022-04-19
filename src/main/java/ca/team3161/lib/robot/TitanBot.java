/*
 * Copyright (c) 2014-2017, FRC3161
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package ca.team3161.lib.robot;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * A subclass of TimedRobot. Autonomous is run in a new Thread, leaving the main robot thread
 * responsible (generally) solely for handling FMS events, Watchdog, etc. This allows
 * autonomous scripts to use convenient semantics such as Thread sleeping rather than periodically
 * checking Timer objects.
 */
public abstract class TitanBot extends TimedRobot implements LifecycleListener {

    private final Lock modeLock = new ReentrantLock();
    private volatile Future<?> autoJob;
    private final LifecycleShifter lifecycleShifter = new LifecycleShifter();
    private final Collection<LifecycleListener> lifecycleAwareComponents = new ArrayList<>();

    /**
     * DO NOT CALL THIS MANUALLY!
     */
    @Override
    public final void robotInit() {
        super.robotInit();
        registerLifecycleComponent(this);
        try {
            robotSetup();
        } catch (Exception e) {
            handleException("Exception in robotSetup", e);
        }
        transitionLifecycle(RobotMode.ON_INIT);
    }

    /**
     * Called once each time the robot is turned on.
     */
    public abstract void robotSetup();

    /**
     * DO NOT CALL THIS MANUALLY!
     * At the start of the autonomous period, start a new background task
     * using the behaviour described in the concrete subclass implementation's
     * autonomousRoutine() method.
     * This new background task allows us to use Thread.sleep rather than a timer,
     * while also not disrupting normal background functions of the
     * robot such as feeding the Watchdog or responding to FMS events.
     * modeLock is used to ensure that the robot is never simultaneously
     * executing both autonomous and teleop routines at the same time.
     */
    @Override
    public final void autonomousInit() {
        super.autonomousInit();
        try {
            modeLock.lockInterruptibly();
            autonomousSetup();
        } catch (final Exception e) {
            handleException("Exception in autonomousSetup", e);
        } finally {
            modeLock.unlock();
        }
        transitionLifecycle(RobotMode.ON_AUTO);
        autoJob = Executors.newSingleThreadExecutor().submit(() -> {
            try {
                modeLock.lockInterruptibly();
                autonomousRoutine(new AutonomousPeriodTimer(getAutonomousPeriodLengthSeconds()));
            } catch (final Exception e) {
                handleException("Exception in autonomousRoutine", e);
            } finally {
                modeLock.unlock();
            }
        });
    }

    /**
     * Called once each time before {@link TitanBot#autonomousRoutine(AutonomousPeriodTimer timer)} is called.
     */
    public abstract void autonomousSetup();

    @Override
    public final void autonomousPeriodic() {
        super.autonomousPeriodic();
    }

    /**
     * The one-shot autonomous "script" to be run in a new Thread.
     *
     * @param timer an {@link AutonomousPeriodTimer} instance to be used for causing the autonomous routine thread to sleep/wait. This should always be used over "bare" Thread.sleep() or Timer.delay().
     * @throws Exception this method failing should never catch the caller unaware - may lead to unpredictable behaviour if so
     */
    public abstract void autonomousRoutine(AutonomousPeriodTimer timer) throws Exception;

    /**
     * DO NOT CALL THIS MANUALLY!
     */
    @Override
    public final void teleopInit() {
        super.teleopInit();
        boolean locked = modeLock.tryLock();
        if (!locked) {
            System.err.println("[ERROR] Failed to run teleopSetup - could not acquire lock!");
            return;
        }
        try {
            teleopSetup();
        } catch (Exception e) {
            handleException("Exception in teleopSetup", e);
        }
        transitionLifecycle(RobotMode.ON_TELEOP);
    }

    /**
     * Called once when the robot enters the teleop mode.
     */
    public abstract void teleopSetup();

    /**
     * DO NOT CALL THIS MANUALLY!
     * Handles running teleopRoutine periodically.
     * Do not override this in subclasses, or else there may be no guarantee
     * that the autonomous thread and the main robot thread, executing teleop
     * code, will not attempt to run concurrently.
     */
    @Override
    public final void teleopPeriodic() {
        super.teleopPeriodic();
        boolean locked = modeLock.tryLock();
        if (!locked) {
            System.err.println("[ERROR] Failed to run teleopRoutine - could not acquire lock!");
            return;
        }
        try {
            teleopRoutine();
        } catch (Exception e) {
            handleException("Exception in teleopRoutine", e);
        } finally {
            modeLock.unlock();
        }
    }

    /**
     * Periodically called during robot teleop mode to enable operator control.
     * This is the only way teleop mode should be handled - do not directly call
     * teleopPeriodic from within this method or unbounded recursion will occur,
     * resulting in a stack overflow and crashed robot code. teleopContinuous
     * is likewise unsupported.
     */
    public abstract void teleopRoutine();

    /**
     * DO NOT CALL THIS MANUALLY!
     */
    @Override
    public final void disabledInit() {
        super.disabledInit();
        boolean locked = modeLock.tryLock();
        if (!locked) {
            System.err.println("[ERROR] Failed to run disabledSetup - could not acquire lock!");
            return;
        }
        try {
            disabledSetup();
        } catch (Exception e) {
            handleException("Exception in disabledSetup", e);
        }
        transitionLifecycle(RobotMode.ON_DISABLED);
    }

    /**
     * Called once when the robot enters the disabled state.
     */
    public abstract void disabledSetup();

    @Override
    public final void disabledPeriodic() {
        super.disabledPeriodic();
        boolean locked = modeLock.tryLock();
        if (!locked) {
            System.err.println("[ERROR] Failed to run disabledRoutine - could not acquire lock!");
            return;
        }
        try {
            disabledRoutine();
        } catch (Exception e) {
            handleException("Exception in disabledRoutine", e);
        } finally {
            modeLock.unlock();
        }
    }

    public abstract void disabledRoutine();

    @Override
    public final void testInit() {
        super.testInit();
        boolean locked = modeLock.tryLock();
        if (!locked) {
            System.err.println("[ERROR] Failed to run testSetup - could not acquire lock!");
            return;
        }
        try {
            testSetup();
        } catch (Exception e) {
            handleException("Exception in testSetup", e);
        }
        transitionLifecycle(RobotMode.ON_TEST);
    }

    public abstract void testSetup();

    @Override
    public final void testPeriodic() {
        super.testPeriodic();
        boolean locked = modeLock.tryLock();
        if (!locked) {
            System.err.println("[ERROR] Failed to run testRoutine - could not acquire lock!");
            return;
        }
        try {
            testRoutine();
        } catch (Exception e) {
            handleException("Exception in testRoutine", e);
        } finally {
            modeLock.unlock();
        }
    }

    public abstract void testRoutine();

    /**
     * Define the length of the Autonomous period, in seconds.
     *
     * @return the length of the Autonomous period, in seconds.
     */
    public abstract int getAutonomousPeriodLengthSeconds();

    public <T extends LifecycleListener> void registerLifecycleComponent(T lifecycleListener) {
        if (!lifecycleAwareComponents.contains(lifecycleListener)) {
            lifecycleAwareComponents.add(lifecycleListener);
        }
    }

    @Override
    public void onLifecycleChange(LifecycleEvent event) {
        switch (event.getMode()) {
            case ON_AUTO:
                break;
            default:
                if (this.autoJob != null) {
                    this.autoJob.cancel(true);
                }
                break;
        }
    }

    private void transitionLifecycle(RobotMode mode) {
        LifecycleEvent event = new LifecycleEvent(mode);
        lifecycleShifter.shift(event);
        lifecycleAwareComponents.forEach(c -> c.onLifecycleChange(event));
    }

    private void handleException(String label, Exception e) {
        DriverStation.reportError(label, e.getStackTrace());
        e.printStackTrace(); // TODO log to file?
    }

    public static class AutonomousPeriodTimer {

        private final long maxPeriodLength;
        private final long startTime = System.currentTimeMillis();
        private volatile RobotMode currentMode = RobotMode.ON_AUTO;

        private AutonomousPeriodTimer(int maxPeriodLengthSeconds) {
            this.maxPeriodLength = TimeUnit.SECONDS.toMillis(maxPeriodLengthSeconds);
        }

        public RobotMode getCurrentMode() {
            return currentMode;
        }

        /*
         * Add a delay to the autonomous routine.
         * This also ensures that the autonomous routine does not continue
         * to run after the FMS notifies us that the autonomous period
         * has ended.
         *
         * @param length how long to wait for (approximate)
         * @param unit   the time units the given delay is in
         * @throws InterruptedException if the autonomous thread is woken up early, for any reason
         * @returns true if there is still expected remaining time in autonomous after the calling thread wakes.
         * If this is false, the calling thread should stop any further execution.
         */
        public boolean waitFor(final long length, final TimeUnit unit) throws InterruptedException {
            Objects.requireNonNull(unit);
            long now = System.currentTimeMillis();
            long endpoint = (startTime + maxPeriodLength);
            long remaining = endpoint - now;
            long toSleep = Math.min(remaining, unit.toMillis(length));
            Thread.sleep(toSleep);
            return System.currentTimeMillis() < endpoint;
        }
    }

    public static class LifecycleShifter {
        private RobotMode previous = RobotMode.NONE;
        private RobotMode current = RobotMode.NONE;

        public void shift(LifecycleEvent event) {
            this.previous = this.current;
            this.current = event.getMode();
        }

        public RobotMode getPrevious() {
            return previous;
        }

        public RobotMode getCurrent() {
            return current;
        }
    }
}
