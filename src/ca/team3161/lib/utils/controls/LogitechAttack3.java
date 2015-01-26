/*
 * Copyright (c) 2014-2015, FRC3161.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *  Redistributions in binary form must reproduce the above copyright notice, this
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
import java.util.stream.Collectors;

public final class LogitechAttack3 extends AbstractController {

    /**
     * {@inheritDoc}.
     */
    public enum LogitechAttack3Control implements Control {
        JOYSTICK;

        @Override
        public int getIdentifier(final Axis axis) {
            return this.ordinal() * LogitechAttack3Control.values().length + axis.getIdentifier();
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

        private LogitechAttack3Button(final int id) {
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
        X,
        Y,
        Z;

        @Override
        public int getIdentifier() {
            return super.ordinal();
        }
    }

    /**
     * Create a new Joystick.
     * @param port the USB port for this Joystick.
     */
    public LogitechAttack3(final int port) {
        super(port, 20, TimeUnit.MILLISECONDS);
        for (final Control control : LogitechAttack3Control.values()) {
            for (final Axis axis : LogitechAttack3Axis.values()) {
                controlsModeMap.put(new ModeIdentifier(control, axis), new LinearJoystickMode());
            }
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getValue(final Control control, final Axis axis) {
        Objects.requireNonNull(control);
        Objects.requireNonNull(axis);
        if (!(control instanceof LogitechAttack3Control)) {
            System.err.println("Joystick on port " + this.port + " getValue() called with invalid control "
                                       + control);
        }
        if (!(axis instanceof LogitechAttack3Axis)) {
            System.err.println("Joystick on port " + this.port + " getValue() called with invalid axis "
                                       + control);
        }
        return controlsModeMap.entrySet()
                       .stream().filter(e -> e.getKey().getControl().equals(control)
                                                     && e.getKey().getAxis().equals(axis))
                       .collect(Collectors.toList()).get(0).getValue().adjust(backingHID.getRawAxis(control.getIdentifier(axis)));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean getButton(final Button button) {
        Objects.requireNonNull(button);
        if (!(button instanceof LogitechAttack3Button)) {
            System.err.println("Joystick on port " + this.port + " getButton() called with invalid button "
                                       + button);
        }
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
    public void setMode(final Control control, final Axis axis, final JoystickMode joystickMode) {
        Objects.requireNonNull(control);
        Objects.requireNonNull(axis);
        Objects.requireNonNull(joystickMode);
        if (!(control instanceof LogitechAttack3Control)) {
            System.err.println("Joystick on port " + this.port + " setMode() called with invalid control "
                                       + control);
        }
        controlsModeMap.put(new ModeIdentifier(control, axis), joystickMode);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void bind(final Button button, final PressType pressType, final Runnable binding) {
        Objects.requireNonNull(button);
        Objects.requireNonNull(pressType);
        Objects.requireNonNull(binding);
        if (!(button instanceof LogitechAttack3Button)) {
            System.err.println("Joystick on port " + this.port + " bind() called with invalid button "
                                       + button);
        }
        buttonBindings.put(new Binding(button, pressType), binding);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void unbind(final Button button, final PressType pressType) {
        Objects.requireNonNull(button);
        Objects.requireNonNull(pressType);
        if (!(button instanceof LogitechAttack3Button)) {
            System.err.println("Joystick on port " + this.port + " unbind() called with invalid button "
                                       + button);
        }
        buttonBindings.entrySet().removeIf(e -> e.getKey().getButton().equals(button)
                                                        && e.getKey().getPressType().equals(pressType));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean hasBinding(final Button button, final PressType pressType) {
        Objects.requireNonNull(button);
        if (!(button instanceof LogitechAttack3Button)) {
            System.err.println("Joystick on port " + this.port + " hasBinding() called with invalid button "
                                       + button);
        }
        return buttonBindings.keySet().stream().anyMatch(b -> b.getButton().equals(button) && b.getPressType().equals(pressType));
    }

}