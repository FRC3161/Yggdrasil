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

import static ca.team3161.lib.utils.Assert.assertTrue;
import static ca.team3161.lib.utils.Utils.normalizePwm;
import static java.util.Objects.requireNonNull;

import ca.team3161.lib.utils.ComposedComponent;
import edu.wpi.first.wpilibj.SpeedController;

import java.util.Collection;
import java.util.Collections;

/**
 * A SpeedController implementation which enforces a maximum change in target speed. When run in a loop,
 * this results in software-controlled acceleration profiles for the composed speed controller.
 */
public class RampingSpeedController implements SpeedController, ComposedComponent<SpeedController> {

    private final SpeedController controller;
    private final double maxStep;
    private final double rampRatio;
    private final double firstFilter;
    private final double secondFilter;
    private boolean inverted;

    private RampingSpeedController(final Builder builder) {
        this.controller = builder.controller;
        this.maxStep = builder.maxStep;
        this.rampRatio = builder.rampRatio;
        this.firstFilter = builder.firstFilter;
        this.secondFilter = builder.secondFilter;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double get() {
        return normalizePwm(controller.get());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void set(final double speed) {
        final double val = normalizePwm(adjust(normalizePwm(speed)));
        controller.set(inverted ? -val : val);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void disable() {
        controller.disable();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Collection<SpeedController> getComposedComponents() {
        return Collections.singleton(controller);
    }

    private double adjust(final double target) {
        final double currentSpeed = get();
        if (Math.abs(target) <= firstFilter) {
            if (Math.abs(target) <= secondFilter) {
                return 0;
            } else {
                return currentSpeed / rampRatio;
            }
        }
        final double error = target - currentSpeed;
        if (error > maxStep) {
            return currentSpeed + maxStep;
        } else if (error < -maxStep) {
            return currentSpeed - maxStep;
        } else {
            return target;
        }
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
        controller.stopMotor();
    }

    /**
     * A Builder for RampingSpeedControllers. Provides an easy way to make RampingSpeedControllers without having to remember
     * parameter order.
     */
    public static class Builder {

        private SpeedController controller;
        private double maxStep = Double.MAX_VALUE;
        private double rampRatio = Double.MAX_VALUE;
        private double firstFilter = Double.MAX_VALUE;
        private double secondFilter = Double.MAX_VALUE;

        /**
         * Use the given parameters to construct a RampingSpeedController.
         * @return a RampingSpeedController instance
         */
        public RampingSpeedController build() {
            verify();
            return new RampingSpeedController(this);
        }

        /**
         * Set the SpeedController to use.
         * @param controller the controller
         * @return this builder
         */
        public Builder controller(SpeedController controller) {
            this.controller = controller;
            return this;
        }

        /**
         * Set the maximum step size to use. Any changes in target speed greater than this will be limited to this.
         * For example, if the composed SpeedController is set to 1.0, the maxStep given is 0.2, and the next call
         * to {@link RampingSpeedController#set(double)} is given the parameter 0.5, then the composed SpeedController
         * will actually be set to 0.8.
         * @param maxStep the maximum step size
         * @return this builder
         */
        public Builder maxStep(double maxStep) {
            this.maxStep = maxStep;
            return this;
        }

        /**
         * Set the ramp ratio. When the controller is between the firstFilter and secondFilter values, this ratio is
         * used to continue governing the stepdown speed to reduce hard braking events.
         * @param rampRatio the rampRatio
         * @return this builder
         */
        public Builder rampRatio(double rampRatio) {
            this.rampRatio = rampRatio;
            return this;
        }

        /**
         * The first filter value. When the new target speed is less than this value, the controller goes into rampdown
         * mode rather than normal operation - maxStep is ignored and either rampRatio is used to compute the next speed,
         * or the controller is stopped entirely.
         * @param firstFilter the first filter
         * @return this builder
         */
        public Builder firstFilter(double firstFilter) {
            this.firstFilter = firstFilter;
            return this;
        }

        /**
         * The second filter value. When the new target speed is less than this value, the controller is immediately set
         * to 0 rather than continuing to slowly ramp down. This is effectively a deadband value.
         * @param secondFilter the second filter
         * @return this builder
         */
        public Builder secondFilter(double secondFilter) {
            this.secondFilter = secondFilter;
            return this;
        }

        private void verify() {
            requireNonNull(controller, "controller cannot be null");
            assertTrue("maxStep cannot be less than -1 or greater than 1", Math.abs(maxStep) <= 1);
            assertTrue("rampRatio must be greater than 1", rampRatio > 1 && rampRatio != Double.MAX_VALUE);
            assertTrue("firstFilter must be between 0 and 1", 0 < firstFilter && firstFilter < 1);
            assertTrue("secondFilter must be greater than 0 and less than firstFilter", 0 < secondFilter && secondFilter < firstFilter);
        }

    }

}
