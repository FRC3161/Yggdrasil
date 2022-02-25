/*
 * Copyright (c) 2014-2017, FRC3161.
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

package ca.team3161.lib.utils.controls;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Function;

import ca.team3161.lib.utils.Utils;

/**
 * A class representing a LogitechAttack3 joystick. Allows for button bindings and axis scaling/curve modes.
 */
public class LogitechAttack3 extends AbstractController {

    /**
     * {@inheritDoc}.
     */
    public enum LogitechAttack3Control implements Control {
        JOYSTICK(0);

        private final int id;

        LogitechAttack3Control(final int id) {
            this.id = id;
        }

        @Override
        public int getIdentifier(final Axis axis) {
            Objects.requireNonNull(axis);
            return id * LogitechAttack3Control.values().length + axis.getIdentifier();
        }
    }

    /**
     * {@inheritDoc}.
     */
    public enum LogitechAttack3Button implements Button {
        TRIGGER(1),
        TOP_CENTER(3),
        TOP_LEFT(4),
        TOP_RIGHT(5),
        TOP_BACK(2),
        BASE_LEFT_FRONT(6),
        BASE_LEFT_BACK(7),
        BASE_BACK_LEFT(8),
        BASE_BACK_RIGHT(9),
        BASE_RIGHT_FRONT(11),
        BASE_RIGHT_BACK(10);

        private final int id;

        LogitechAttack3Button(final int id) {
            this.id = id;
        }

        @Override
        public int getIdentifier() {
            return this.id;
        }
    }

    /**
     * {@inheritDoc}.
     */
    public enum LogitechAttack3Axis implements Axis {
        X(0),
        Y(1),
        Z(2);

        private final int id;

        LogitechAttack3Axis(final int id) {
            this.id = id;
        }

        @Override
        public int getIdentifier() {
            return id;
        }
    }

    /**
     * Create a new Joystick.
     *
     * @param port the USB port for this Joystick.
     */
    public LogitechAttack3(final int port) {
        this(port, 20, TimeUnit.MILLISECONDS);
    }

    /**
     * Create a new Joystick, with a specific polling frequency (for button bindings).
     * For example, to poll at 50Hz, you might use a period of 20 and a timeUnit of TimeUnit.MILLISECONDS.
     *
     * @param port     the USB port for this Joystick.
     * @param period   the timeout period between button mapping polls.
     * @param timeUnit the unit of the timeout period.
     */
    public LogitechAttack3(final int port, final int period, final TimeUnit timeUnit) {
        super(port, period, timeUnit);
        for (final Control control : LogitechAttack3Control.values()) {
            for (final Axis axis : LogitechAttack3Axis.values()) {
                controlsModeMap.put(new Mapping(control, axis), new LinearJoystickMode());
            }
        }
    }

    protected static void validate(final Mapping mapping, final String message) {
        Objects.requireNonNull(mapping);
        if (!(mapping.getControl() instanceof LogitechAttack3Control) || !(mapping.getAxis() instanceof LogitechAttack3Axis)) {
            throw new IllegalArgumentException(message);
        }
    }

    protected static void validate(final Button button, final String message) {
        Objects.requireNonNull(button);
        if (!(button instanceof LogitechAttack3Button)) {
            throw new IllegalArgumentException(message);
        }
    }

    protected static void validate(final Binding binding, final String message) {
        Objects.requireNonNull(binding);
        binding.getButtons().stream().forEach((Gamepad.Button button) -> {
            if (!(button instanceof LogitechAttack3Button)) {
                throw new IllegalArgumentException(message);
            }
        });
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getValue(final Mapping mapping) {
        validate(mapping, "Joystick on port " + this.port + " getValue() called with invalid mapping " + mapping);
        return controlsModeMap.get(mapping).apply(backingHID.getRawAxis(mapping.getControl().getIdentifier(mapping.getAxis())));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean getButton(final Button button) {
        validate(button, "Joystick on port " + this.port + " getButton() called with invalid button " + button);
        return backingHID.getRawButton(button.getIdentifier());
    }

    public int getDpad() {
        return backingHID.getPOV();
    }

    /**
     *
     */
    @Override
    protected Set<Button> getButtons() {
        return new HashSet<>(Arrays.asList(LogitechAttack3Button.values()));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setMode(final Mapping mapping, final Function<Double, Double> function) {
        Objects.requireNonNull(function);
        validate(mapping, "Joystick on port " + this.port + " setMode() called with invalid mapping " + mapping);
        controlsModeMap.put(mapping, function);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setMode(final Function<Double, Double> function) {
        for (LogitechAttack3Control control : LogitechAttack3Control.values()) {
            for (LogitechAttack3Axis axis : LogitechAttack3Axis.values()) {
                setMode(control, axis, function);
            }
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void map(final Mapping mapping, final Consumer<Double> consumer) {
        Objects.requireNonNull(consumer);
        validate(mapping, "Gamepad on port " + this.port + " getValue() called with invalid mapping " + mapping);
        controlsMapping.put(mapping, Utils.safeExec(mapping.toString(), consumer));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void bind(final Binding binding, final Runnable action) {
        Objects.requireNonNull(action);
        validate(binding, "Joystick on port " + this.port + " bind() called with invalid binding " + binding);
        buttonBindings.put(binding, () -> Utils.safeExec(binding.toString(), action));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void unbind(final Binding binding) {
        validate(binding, "Joystick on port " + this.port + " unbind() called with invalid binding " + binding);
        buttonBindings.remove(binding);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean hasBinding(final Binding binding) {
        validate(binding, "Joystick on port " + this.port + " hasBinding() called with invalid binding " + binding);
        return buttonBindings.containsKey(binding);
    }

}
