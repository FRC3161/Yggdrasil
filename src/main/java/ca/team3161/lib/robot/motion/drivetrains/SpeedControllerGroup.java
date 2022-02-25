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

package ca.team3161.lib.robot.motion.drivetrains;

import ca.team3161.lib.utils.Assert;
import ca.team3161.lib.utils.ComposedComponent;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.wpilibj.SpeedController;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Implements a container for SpeedControllers.
 */
public final class SpeedControllerGroup implements SpeedController, ComposedComponent<SpeedController> {

    private static final float INVERTED = -1.0f;
    private static final float NON_INVERTED = 1.0f;

    private final Set<SpeedController> speedControllers = new HashSet<>();
    private float inversion = NON_INVERTED;

    /**
     * Create a new SpeedControllerGroup instance.
     *
     * @param controllers a varargs list or array of SpeedController objects. May be
     *                    all the same type, or may be mixed.
     */
    public SpeedControllerGroup(final SpeedController... controllers) {
        this(Arrays.asList(controllers));
    }

    /**
     * Create a new SpeedControllerGroup instance.
     *
     * @param controllers a collection of SpeedControllers for this SpeedControllerGroup to manage
     */
    public SpeedControllerGroup(final Collection<SpeedController> controllers) {
        Objects.requireNonNull(controllers);
        Assert.assertTrue("Must have at least one SpeedController per SpeedControllerGroup", controllers.size() > 0);
        speedControllers.addAll(controllers);
    }

    /**
     * Invert all PWM values for this SpeedControllerGroup.
     *
     * @param inverted whether the PWM values should be inverted or not
     */
    public void setInverted(final boolean inverted) {
        if (inverted) {
            inversion = INVERTED;
        } else {
            inversion = NON_INVERTED;
        }
    }

    /**
     * Same as {@link SpeedControllerGroup#setInverted(boolean)}, but returns "this" rather than void.
     * @param inversion whether the PWM values should be inverted or not
     * @return this
     */
    public SpeedControllerGroup setInversion(final boolean inversion) {
        setInverted(inversion);
        return this;
    }

    /**
     * The current speed of this SpeedControllerGroup.
     *
     * @return the current PWM value of the SpeedController collection (-1.0 to 1.0)
     */
    @Override
    public double get() {
        // All of the SpeedControllers will always be set to the same value,
        // so simply get the value of the first one.
        return inversion * speedControllers.stream().findFirst()
                .flatMap(controller -> Optional.ofNullable(controller.get()))
                .orElse(0.0);
    }

    /**
     * Return the list of SpeedControllers which this SpeedControllerGroup was constructed with.
     *
     * @return the SpeedControllers.
     */
    @Override
    public Collection<SpeedController> getComposedComponents() {
        return new HashSet<>(speedControllers);
    }

    /**
     * The speeds of all SpeedControllers within this SpeedControllerGroup.
     * They should all be nearly identical, other than error due to floating point
     * precision.
     *
     * @return a list enumerating all the current PWM values of the SpeedController collection (-1.0 to 1.0)
     */
    public List<Double> getAll() {
        return speedControllers.stream().mapToDouble(SpeedController::get).boxed().collect(Collectors.toList());
    }

    /**
     * Set the pwm value (-1.0 to 1.0).
     *
     * @param pwm the PWM value to assign to each SpeedController in the collection
     */
    @Override
    public void set(final double pwm) {
        speedControllers.forEach(c -> c.set(inversion * Utils.normalizePwm(pwm)));
    }

    /**
     * Disable each SpeedController in the collection.
     */
    @Override
    public void disable() {
        speedControllers.forEach(SpeedController::disable);
    }

    @Override
    public boolean getInverted() {
        return inversion == INVERTED;
    }

    @Override
    public void stopMotor() {
        speedControllers.forEach(SpeedController::stopMotor);
    }
}
