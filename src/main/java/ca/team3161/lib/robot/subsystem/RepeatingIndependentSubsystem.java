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

import static ca.team3161.lib.utils.Utils.requireNonNegative;
import static java.util.Objects.requireNonNull;

import java.util.concurrent.TimeUnit;

/**
 * A Subsystem whose task is run repeatedly with a specified period, until cancelled.
 */
public abstract class RepeatingIndependentSubsystem extends AbstractIndependentSubsystem {

    private final long timeout;
    private final TimeUnit timeUnit;

    public RepeatingIndependentSubsystem() {
        this(20, TimeUnit.MILLISECONDS);
    }

    public RepeatingIndependentSubsystem(final long timeout, final TimeUnit timeUnit) {
        requireNonNull(timeUnit);
        this.timeout = requireNonNegative(timeout);
        this.timeUnit = requireNonNull(timeUnit);
    }

    @Override
    public void start() {
        if (job != null) {
            job.cancel(true);
        }
        job = getExecutorService().scheduleAtFixedRate(new RunTask(timeUnit.toMillis(timeout)), 0L, timeout, timeUnit);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean isDone() {
        return false;
    }
}
