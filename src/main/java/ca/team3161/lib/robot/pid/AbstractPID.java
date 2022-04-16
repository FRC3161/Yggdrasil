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

package ca.team3161.lib.robot.pid;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.wpilibj.PIDSource;
import java.util.concurrent.TimeUnit;

/**
 * A PID loop, which uses a PIDSrc and a set of constants to iteratively determine
 * output values with which a system can reach and maintain a target value.
 *
 * @param <T> the type of sensor used as input to this PID system.
 * @param <V> the type of value read from the sensor used as input to this PID system.
 */
public abstract class AbstractPID<T extends PIDSource, V extends Number> implements PID<T, V> {

    /**
     * A PIDSrc sensor.
     */
    protected final PIDSrc<T, V> source;

    protected volatile float deadband;
    protected volatile float kP;
    protected volatile float kI;
    protected volatile float kD;

    protected volatile float integralError;
    protected volatile float prevError;
    protected volatile float deltaError;

    protected volatile int deadbandPeriod;
    protected volatile TimeUnit deadbandUnit;
    protected volatile long lastTimeNotAtTarget;

    /**
     * Create a new AbstractPID instance.
     *
     * @param source         the PIDSrc source sensor
     * @param deadband       filter value - do not act when current error is within this bound. This can be disabled by passing a negative value
     * @param deadbandPeriod the amount of time to remain within acceptable error of the target value before claiming to actually be at the target
     * @param deadbandUnit   the units for deadbandPeriod
     * @param kP             P constant
     * @param kI             I constant
     * @param kD             D constant
     */
    public AbstractPID(final PIDSrc<T, V> source, final float deadband,
                       final int deadbandPeriod, final TimeUnit deadbandUnit,
                       final float kP, final float kI, final float kD) {
        this.source = requireNonNull(source);
        this.deadband = deadband;
        this.deadbandPeriod = deadbandPeriod;
        this.deadbandUnit = requireNonNull(deadbandUnit);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void clear() {
        integralError = 0.0f;
        prevError = 0.0f;
        deltaError = 0.0f;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public abstract V pid(final V target);

    /**
     * {@inheritDoc}
     */
    @Override
    public final PIDSrc<T, V> getSrc() {
        return this.source;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final boolean atTarget() {
        if (deadband < 0 || deadbandPeriod < 0 || deadbandUnit == null) {
            return false;
        }
        final boolean atTarget = Math.abs(prevError) < deadband;
        final long timeNow = System.currentTimeMillis();
        if (!atTarget) {
            lastTimeNotAtTarget = timeNow;
        }
        final boolean deadbandPeriodElapsed = lastTimeNotAtTarget < timeNow - deadbandUnit.toMillis(deadbandPeriod);

        return atTarget && deadbandPeriodElapsed;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setkP(final float kP) {
        this.kP = kP;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public float getkP() {
        return kP;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setkI(final float kI) {
        this.kI = kI;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public float getkI() {
        return kI;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setkD(final float kD) {
        this.kD = kD;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public float getkD() {
        return kD;
    }
}
