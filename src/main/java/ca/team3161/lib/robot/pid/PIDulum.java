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

import static ca.team3161.lib.utils.Utils.normalizePwm;

import ca.team3161.lib.utils.ComposedComponent;
import java.util.Collection;
import java.util.Collections;
import java.util.concurrent.TimeUnit;

/**
 * A PID controller for inverted pendulum systems (PID pendulum... getPIDValue it?)
 *
 * @param <V> the specific source type which provides angle measurements for this PIDulum.
 */
public final class PIDulum<V extends PIDAngleValueSrc<?>> extends AbstractPID<V, Float> implements ComposedComponent<PIDSrc<V, Float>> {

    private final float offsetAngle;
    private final float torqueConstant;

    /**
     * @param source         the PIDSrc source sensor
     * @param deadband       filter value - do not act when current error is within this bound
     * @param deadbandPeriod the amount of time to remain within acceptable error of the target value before claiming to actually be at the target
     * @param deadbandUnit   the units for deadbandPeriod
     * @param kP             P constant
     * @param kI             I constant
     * @param kD             D constant
     * @param offsetAngle    the balance point of the inverted pendulum
     * @param torqueConstant "feed forward" term constant to allow the pendulum to hold position against gravity
     */
    public PIDulum(final PIDSrc<V, Float> source, final float deadband,
                   final int deadbandPeriod, final TimeUnit deadbandUnit,
                   final float kP, final float kI, final float kD,
                   final float offsetAngle, final float torqueConstant) {
        super(source, deadband, deadbandPeriod, deadbandUnit, kP, kI, kD);
        this.offsetAngle = offsetAngle;
        this.torqueConstant = torqueConstant;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Float pid(final Float target) {
        float kErr;
        float pOut;
        float iOut;
        float dOut;
        float feedForward;
        float output;

        kErr = target - source.getPIDValue();

        deltaError = prevError - kErr;
        prevError = kErr;
        integralError += kErr;

        pOut = kErr * kP;
        iOut = integralError * kI;
        dOut = deltaError * kD;

        if (iOut > 1) {
            iOut = 1;
        }

        feedForward = torqueConstant * (source.getPIDValue() - offsetAngle);

        if (atTarget()) {
            return feedForward;
        }

        output = pOut + iOut + dOut + feedForward;

        return normalizePwm(output);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Collection<PIDSrc<V, Float>> getComposedComponents() {
        return Collections.singleton(source);
    }
}
