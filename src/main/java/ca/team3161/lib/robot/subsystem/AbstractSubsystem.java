/*
 * Copyright (c) 2015-2017, FRC3161.
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

package ca.team3161.lib.robot.subsystem;

import ca.team3161.lib.utils.ComposedComponent;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashSet;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.locks.Lock;

/**
 * An abstract implementation of the Subsystem interface.
 * <p>
 * Manages acquiring and releasing required resources and handles starting/stopping
 * tasks. Subclasses are expected to define the semantics of how tasks are run
 * (shared or independent workers, one-shot or repeating, etc).
 */
public abstract class AbstractSubsystem implements Subsystem {

    // reduced priority to alleviate starvation of main robot thread for watchdog, network, etc
    protected static final int THREAD_PRIORITY = Thread.NORM_PRIORITY - 1;

    /**
     * A list of resourceLocks which this Subsystem requires.
     *
     * @see ResourceTracker
     */
    protected final Set<Lock> resourceLocks = new HashSet<>();

    /**
     * The Future representing this Subsystem's task. 'null' if the subsystem
     * has never yet been started. This can be used to check if the subsystem
     * has ever been started, or to cancel scheduled jobs.
     *
     * @see Subsystem#start()
     */
    protected Future<?> job;

    /**
     * {@inheritDoc}
     */
    @Override
    @SuppressWarnings("unchecked")
    public final void require(final Object resource) {
        Objects.requireNonNull(resource);
        boolean alreadyTracked = !resourceLocks.add(ResourceTracker.track(resource));
        if (!alreadyTracked && (resource instanceof ComposedComponent)) {
            ComposedComponent cc = (ComposedComponent) resource;
            cc.getComposedComponents().forEach(this::require);
        }
    }

    /**
     * Helper method to acquire all defined resources.
     *
     * @throws InterruptedException if interrupted while holding any resource lock
     */
    protected final void acquireResources() throws InterruptedException {
        resourceLocks.forEach(Lock::tryLock);
    }

    /**
     * Helper method to release all defined resources.
     */
    protected final void releaseResources() {
        resourceLocks.forEach(Lock::unlock);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final Future<?> getJob() {
        return job;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final boolean isCancelled() {
        return (getJob() != null && getJob().isCancelled());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final boolean isStarted() {
        return getJob() != null;
    }

    @Override
    public final boolean isScheduled() {
        return isStarted() && !isCancelled() && !isDone();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void cancel() {
        if (getJob() != null) {
            getJob().cancel(true);
        }
    }

    /**
     * The task for this Subsystem to run.
     */
    protected class RunTask implements Runnable {
        @Override
        public void run() {
            try {
                acquireResources();
                task();
            } catch (final Exception e) {
                DriverStation.reportError(e.getMessage(), e.getStackTrace());
                e.printStackTrace();
            } finally {
                releaseResources();
            }
        }
    }

    /**
     * Get the executor service which executes this subsystem's tasks.
     * This service may or may not be shared with other subsystems;
     * for Independent subsystems it is never shared, and for Pooled
     * subsystems it is always shared.
     *
     * @return the executor service.
     * @see AbstractIndependentSubsystem
     * @see AbstractPooledSubsystem
     */
    protected abstract ScheduledExecutorService getExecutorService();

}
