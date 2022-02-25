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

package ca.team3161.lib.robot.pid;

import static ca.team3161.lib.utils.Utils.normalizePwm;
import static ca.team3161.lib.utils.Utils.requireNonNegative;
import static java.util.Objects.requireNonNull;

import java.util.Arrays;
import java.util.Collection;
import java.util.concurrent.TimeUnit;

import ca.team3161.lib.utils.ComposedComponent;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.pidwrappers.PIDEncoder;

/**
 * A SpeedController implementation which treats its input and output values as proportions of PID velocity targets,
 * using an Encoder to measure the rotational rate of the associated SpeedController (ex Talon, Victor, Jaguar).
 * Intended usage is to call {@link ca.team3161.lib.robot.pid.VelocityController#set(double)} whenever the target
 * value changes OR a PID loop iteration is desired. {@link ca.team3161.lib.robot.pid.VelocityController#pid(Float)}
 * will only return the adjusted value for the next PID iteration; this value represents an actual motor output value,
 * but is not automatically applied to the backing SpeedController instance.
 */
public class VelocityController extends AbstractPID<PIDEncoder, Float> implements SpeedController, PIDRateValueSrc<PIDEncoder>, ComposedComponent<Object> {

    protected final SpeedController speedController;
    protected float maxRotationalRate = 0;
    protected float target = 0;
    protected final float maxIntegralError;
    protected final float deadband;
    protected boolean inverted;

    /**
     * Construct a new VelocityController instance.
     *
     * @param speedController   a backing SpeedController (ex physical Jaguar, Talon, Victor).
     * @param encoder           an Encoder which measures the output of the associated physical SpeedController.
     * @param maxRotationalRate the maximum rotational rate as reported by the Encoder.
     * @param kP                the Proportional PID constant.
     * @param kI                the Integral PID constant.
     * @param kD                the Derivative PID constant.
     * @param maxIntegralError  limit constant for the integral error.
     * @param deadband          if the absolute value of the deadband falls within this range, output 0.
     */
    public VelocityController(final SpeedController speedController, final PIDEncoder encoder, final float maxRotationalRate,
                              final float kP, final float kI, final float kD, final float maxIntegralError, final float deadband) {
        this(speedController, new EncoderPIDSrc(encoder), maxRotationalRate, kP, kI, kD, maxIntegralError, deadband);
    }

    /**
     * Construct a new VelocityController instance.
     *
     * @param speedController   a backing SpeedController (ex physical Jaguar, Talon, Victor).
     * @param encoderPidSrc     an EncoderPidSrc which measures the output of the associated physical SpeedController.
     * @param maxRotationalRate the maximum rotational rate as reported by the Encoder.
     * @param kP                the Proportional PID constant.
     * @param kI                the Integral PID constant.
     * @param kD                the Derivative PID constant.
     * @param maxIntegralError  limit constant for the integral error.
     * @param deadband          if the absolute value of the deadband falls within this range, output 0.
     */
    public VelocityController(final SpeedController speedController, final PIDRateValueSrc<PIDEncoder> encoderPidSrc, final float maxRotationalRate,
                              final float kP, final float kI, final float kD, final float maxIntegralError, final float deadband) {
        super(encoderPidSrc, -1, -1, TimeUnit.MILLISECONDS, kP, kI, kD);
        this.maxRotationalRate = maxRotationalRate;
        this.speedController = requireNonNull(speedController);
        this.maxIntegralError = maxIntegralError;
        this.deadband = requireNonNegative(deadband);
    }

    /**
     * Get the target rotational rate proportion which this VelocityController is set to.
     *
     * @return the proportional rotational rate target.
     */
    @Override
    public double get() {
        return target;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Float getRate() {
        return source.getPIDValue();
    }

    /**
     * Set the target rotational rate of this VelocityController.
     * This method should be called very frequently, as the PID loop only iterates when this method is called.
     * @param v target value.
     */
    @Override
    public void set(final double v) {
        this.target = inverted ? (float) -v : (float) v;
        speedController.set(pid(target * maxRotationalRate));
    }

    /**
     * Disable this VelocityController.
     */
    @Override
    public void disable() {
        clear();
        speedController.disable();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Collection<Object> getComposedComponents() {
        return Arrays.asList(speedController, source);
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
        float output;

        kErr = target - source.getPIDValue();

        deltaError = prevError - kErr;
        prevError = kErr;
        integralError += kErr;

        if (integralError > maxIntegralError) {
            integralError = maxIntegralError;
        } else if (integralError < -maxIntegralError) {
            integralError = -maxIntegralError;
        }

        if (Math.abs(target) <= deadband) {
            integralError = 0;
        }

        pOut = kErr * kP;
        iOut = integralError * kI;
        dOut = deltaError * kD;

        if (iOut > 1.0f) {
            iOut = 1.0f;
        }

        output = pOut + iOut + dOut;

        return normalizePwm(output);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public PIDEncoder getSensor() {
        return source.getSensor();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Float getPIDValue() {
        return getRate();
    }

    @Override
    public void setPIDSourceType(final PIDSourceType pidSourceType) {
        throw new UnsupportedOperationException();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kRate;
    }

    @Override
    public void setInverted(final boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public void stopMotor() {
        speedController.stopMotor();
    }
}
