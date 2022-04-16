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

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

/**
 * Abstracts a system which uses resourceLocks and has some task (recurring or
 * one-shot) to be performed. An example is PID control - monitor sensors
 * and periodically set motor values based on this. Independent subsystems do not
 * share a common work queue, and so independent subsystems are suitable for tasks
 * which contain long-running operations (which includes any Thread.sleeps,
 * Timers, while(true) loops, etc), as this will not affect other Subsystems' execution.
 * However, there is more overhead involved with this. If you do not need a Subsystem
 * which is able to execute long-running operations without interfering with
 * other Subsystems, use a PooledSubsystem so that the workload can be shared among
 * threads.
 *
 * @see AbstractPooledSubsystem
 */
public abstract class AbstractIndependentSubsystem extends AbstractSubsystem {

    private final ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread t = new Thread(r, getClass().getSimpleName() + "Thread");
        t.setPriority(AbstractSubsystem.THREAD_PRIORITY);
        return t;
    });

    @Override
    public ScheduledExecutorService getExecutorService() {
        return executor;
    }

}
