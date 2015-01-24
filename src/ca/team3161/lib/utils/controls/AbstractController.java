/*
 * Copyright (c) 2015, FRC3161.
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

import ca.team3161.lib.robot.RepeatingSubsystem;
import ca.team3161.lib.utils.Assert;
import edu.wpi.first.wpilibj.GenericHID;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;

public abstract class AbstractController extends RepeatingSubsystem implements Gamepad {

    /* The actual FIRST-provided input device that we are implementing a
    * convenience wrapper around.
    */
    protected final GenericHID backingHID;
    protected final Map<ModeIdentifier, JoystickMode> controlsModeMap = new HashMap<>();
    protected final Map<Binding, Runnable> buttonBindings = new ConcurrentHashMap<>();
    protected final Map<Button, Boolean> buttonStates = new ConcurrentHashMap<>();
    protected final int port;

    protected AbstractController(final int port, final long timeout, final TimeUnit timeUnit) {
        super(timeout, timeUnit);
        Assert.assertTrue(port >= 0);
        this.port = port;
        backingHID = new edu.wpi.first.wpilibj.Joystick(port); // Joystick happens to work well here, but any GenericHID should be fine
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public GenericHID getBackingHID() {
        return backingHID;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int getPort() {
        return this.port;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void enableBindings() {
        start();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void disableBindings() {
        cancel();
    }

    @Override
    protected void defineResources() {
        // none!
    }

    /**
     * Get the set of Buttons on this controller
     * @return a set of Buttons
     */
    protected abstract Set<Button> getButtons();

    @Override
    protected void task() throws Exception {
        final Map<Button, Boolean> previousButtonStates = new HashMap<>(buttonStates);
        for (final Button button : getButtons()) {
            buttonStates.put(button, getButton(button));
        }
        synchronized (buttonBindings) {
            for (final Map.Entry<Binding, Runnable> binding : buttonBindings.entrySet()) {
                final Button button = binding.getKey().getButton();
                final PressType pressType = binding.getKey().getPressType();
                final Runnable action = binding.getValue();
                switch (pressType) {
                    case PRESS:
                        if (buttonStates.get(button) && !previousButtonStates.get(button)) {
                            action.run();
                        }
                        break;
                    case RELEASE:
                        if (!buttonStates.get(button) && previousButtonStates.get(button)) {
                            action.run();
                        }
                        break;
                    case HOLD:
                        if (buttonStates.get(button)) {
                            action.run();
                        }
                        break;
                    default:
                        System.err.println("Gamepad on port " + Integer.toString(getPort())
                                                   + " has binding for unknown button press type " + pressType);
                        break;
                }
            }
        }
    }

    protected static class Binding {
        private final Button button;
        private final PressType pressType;

        public Binding(final Button button, final PressType pressType) {
            Objects.requireNonNull(button);
            Objects.requireNonNull(pressType);
            this.button = button;
            this.pressType = pressType;
        }

        public Button getButton() {
            return button;
        }

        public PressType getPressType() {
            return pressType;
        }

        @Override
        public boolean equals(final Object o) {
            if (this == o) {
                return true;
            }
            if (o == null || getClass() != o.getClass()) {
                return false;
            }

            final Binding binding = (Binding) o;

            if (!button.equals(binding.button)) {
                return false;
            }
            if (pressType != binding.pressType) {
                return false;
            }

            return true;
        }

        @Override
        public int hashCode() {
            int result = button.hashCode();
            result = 31 * result + pressType.hashCode();
            return result;
        }
    }

    protected static class ModeIdentifier {
        private final Control control;
        private final Axis axis;

        public ModeIdentifier(final Control control, final Axis axis) {
            Objects.requireNonNull(control);
            Objects.requireNonNull(axis);
            this.control = control;
            this.axis = axis;
        }

        public Control getControl() {
            return control;
        }

        public Axis getAxis() {
            return axis;
        }

        @Override
        public boolean equals(final Object o) {
            if (this == o) {
                return true;
            }
            if (o == null || getClass() != o.getClass()) {
                return false;
            }

            final ModeIdentifier that = (ModeIdentifier) o;

            if (!axis.equals(that.axis)) {
                return false;
            }
            if (!control.equals(that.control)) {
                return false;
            }

            return true;
        }

        @Override
        public int hashCode() {
            int result = control.hashCode();
            result = 31 * result + axis.hashCode();
            return result;
        }
    }
}