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

package ca.team3161.lib.utils.controls;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.wpilibj.GenericHID;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Stream;

/**
 * An interface defining a Gamepad controller. All Gamepads are expected to
 * have two thumbsticks, a directional pad, and some clickable buttons.
 * Not all Gamepads will have variable triggers, and not all raw button or
 * axis mappings are the same. These details are left to specific Gamepad
 * implementations.
 */
public interface Gamepad {

    /**
     * Get the backing input device of this Gamepad.
     *
     * @return the backing input device, eg Joystick
     */
    GenericHID getBackingHID();

    /**
     * Get the USB port (as numbered in the Driver Station) that this Gamepad is plugged into.
     *
     * @return the USB port number
     */
    int getPort();

    /**
     * Get the value of an axis on a control. Generally between -1.0 and 1.0. Controls
     * and Axes should be provided by Gamepad implementations supplying their own valid
     * possible values. Controls and Axes defined by one Gamepad implementation should
     * not be used as parameters to other Gamepad implementations.
     *
     * @param mapping the mapping to check
     * @return the value of the axis on the control
     */
    double getValue(Mapping mapping);

    default double getValue(Control control, Axis axis) {
        return getValue(new Mapping(control, axis));
    }

    /**
     * Get the value of a button on the controller. Buttons
     * should be provided by Gamepad implementations supplying their own valid
     * possible values. Buttons defined by one Gamepad implementation should
     * not be used as parameters to other Gamepad implementations.
     *
     * @param button which button to check. The mapping from values here to
     *               actual buttons will depend on the specific Gamepad implementation
     * @return whether the specified button is currently pressed or not
     */
    boolean getButton(Button button);

    /**
     * Set a mode to adjust input on one of the controls of this Gamepad. Controls
     * should be provided by Gamepad implementations supplying their own valid
     * possible values. Controls defined by one Gamepad implementation should
     * not be used as parameters to other Gamepad implementations. Likewise for axes.
     *
     * @param control      the control on which to set a mode
     * @param axis         the axis of the control on which to set a mode
     * @param joystickMode the mode to set
     */
    default void setMode(Control control, Axis axis, JoystickMode joystickMode) {
        setMode(new Mapping(control, axis), joystickMode);
    }

    /**
     * Set a function to adjust input on one of the controls of this Gamepad. Controls
     * should be provided by Gamepad implementations supplying their own valid
     * possible values. Controls defined by one Gamepad implementation should
     * not be used as parameters to other Gamepad implementations. Likewise for axes.
     *
     * @param control  the control on which to set a mode
     * @param axis     the axis of the control on which to set a mode
     * @param function the function to apply
     */
    default void setMode(Control control, Axis axis, Function<Double, Double> function) {
        setMode(new Mapping(control, axis), function);
    }

    /**
     * Set a function to adjust input on one of the controls of this Gamepad. Controls
     * should be provided by Gamepad implementations supplying their own valid
     * possible values. Controls defined by one Gamepad implementation should
     * not be used as parameters to other Gamepad implementations. Likewise for axes.
     *
     * @param mapping  the mapping for which to set a mode
     * @param function the function to apply
     */
    void setMode(Mapping mapping, Function<Double, Double> function);

    /**
     * Set a function to adjust input on one of the controls of this Gamepad. Controls
     * should be provided by Gamepad implementations supplying their own valid
     * possible values. Controls defined by one Gamepad implementation should
     * not be used as parameters to other Gamepad implementations. Likewise for axes.
     *
     * @param function the function to apply to all controls and all axes of this Gamepad
     */
    void setMode(Function<Double, Double> function);

    /**
     * Map a control/axis pair to a function.
     * <p>
     * The given function will be periodically called and given the then-current value of
     * the control/axis pair specified. This is similar to binding button presses to methods,
     * on a continuous-valued input (eg thumbstick) instead, to a method which requires a double
     * value, for example tank drive.
     *
     * @param mapping  the control mapping to use
     * @param consumer the function to be called
     */
    void map(Mapping mapping, Consumer<Double> consumer);

    default void map(Control control, Axis axis, Consumer<Double> consumer) {
        map(new Mapping(control, axis), consumer);
    }

    /**
     * Bind a button press on this gamepad to an action to be performed when the button
     * is pressed. Buttons should be provided by Gamepad implementations supplying their
     * own valid possible values. Buttons defined by one Gamepad implementation should
     * not be used as parameters to other Gamepad implementations.
     *
     * @param button  the button on which to bind an action
     * @param binding the action to be bound
     */
    default void bind(Button button, Runnable binding) {
        bind(button, PressType.PRESS, binding);
    }

    default void bind(Button button, Consumer<Boolean> binding) {
        bind(button, PressType.PRESS, () -> binding.accept(true));
        bind(button, PressType.RELEASE, () -> binding.accept(false));
    }

    /**
     * Bind a button press on this gamepad to an action to be performed when the button
     * is pressed, released, or periodically while the button is held. Buttons should be
     * provided by Gamepad implementations supplying their own valid possible values.
     * Buttons defined by one Gamepad implementation should not be used as parameters
     * to other Gamepad implementations.
     *
     * @param button    the button on which to bind an action
     * @param pressType the type of button press which should trigger the action
     * @param binding   the action to be bound
     */
    default void bind(Button button, PressType pressType, Runnable binding) {
        bind(Collections.singleton(button), pressType, binding);
    }

    /**
     * Bind a button combination press on this gamepad to an action to be performed when the button
     * is pressed, released, or periodically while the button is held. Buttons should be
     * provided by Gamepad implementations supplying their own valid possible values.
     * Buttons defined by one Gamepad implementation should not be used as parameters
     * to other Gamepad implementations.
     *
     * @param buttons   the button combination on which to bind an action
     * @param pressType the type of button press which should trigger the action
     * @param binding   the action to be bound
     */
    default void bind(Set<Button> buttons, PressType pressType, Runnable binding) {
        bind(new Binding(buttons, pressType), binding);
    }

    /**
     * Bind a button press on this gamepad to an action to be performed when the button
     * is pressed, released, or periodically while the button is held. Buttons should be
     * provided by Gamepad implementations supplying their own valid possible values.
     * Buttons defined by one Gamepad implementation should not be used as parameters
     * to other Gamepad implementations.
     *
     * @param binding the binding on which to bind an action
     * @param action the action to be bound
     */
    void bind(Binding binding, Runnable action);

    /**
     * Remove all bindings for the given button. Buttons should be provided by Gamepad
     * implementations supplying their own valid possible values. Buttons defined by one
     * Gamepad implementation should not be used as parameters to other Gamepad implementations.
     *
     * @param button the button for which to unbind all actions
     */
    default void unbind(Button button) {
        Stream.of(PressType.values()).forEach(t -> unbind(button, t));
    }

    /**
     * Remove a binding for the given button. Buttons should be provided by Gamepad
     * implementations supplying their own valid possible values. Buttons defined by one
     * Gamepad implementation should not be used as parameters to other Gamepad implementations.
     *
     * @param button    the button for which to unbind an action
     * @param pressType the type of button press for which to unbind an action
     */
    default void unbind(Button button, PressType pressType) {
        unbind(Collections.singleton(button), pressType);
    }

    /**
     * Remove a binding for the given button combination. Buttons should be provided by Gamepad
     * implementations supplying their own valid possible values. Buttons defined by one
     * Gamepad implementation should not be used as parameters to other Gamepad implementations.
     *
     * @param buttons   the button combination for which to unbind an action
     * @param pressType the type of button press for which to unbind an action
     */
    default void unbind(Set<Button> buttons, PressType pressType) {
        unbind(new Binding(buttons, pressType));
    }

    /**
     * Remove a binding for the given binding.
     *
     * @param binding the binding for which to unbind an action
     */
    void unbind(Binding binding);

    /**
     * Check if a given button has a bound action. Buttons should be provided by Gamepad
     * implementations supplying their own valid possible values. Buttons defined by one
     * Gamepad implementation should not be used as parameters to other Gamepad implementations.
     *
     * @param button the button to check for any bindings
     * @return if the button has any bindings
     */
    default boolean hasBinding(Button button) {
        return Stream.of(PressType.values()).map(pressType -> hasBinding(button, pressType)).anyMatch(t -> t);
    }

    /**
     * Check if a given button has a bound action for a specific button press type. Buttons
     * should be provided by Gamepad implementations supplying their own valid possible values.
     * Buttons defined by one Gamepad implementation should not be used as parameters to other
     * Gamepad implementations.
     *
     * @param button    the button to check for bindings
     * @param pressType the type of button press to check for bindings
     * @return if the button has any bindings for the given press type
     */
    default boolean hasBinding(Button button, PressType pressType) {
        return hasBinding(Collections.singleton(button), pressType);
    }

    default boolean hasBinding(Set<Button> buttons, PressType pressType) {
        return hasBinding(new Binding(buttons, pressType));
    }

    boolean hasBinding(Binding binding);

    /**
     * Enable button bindings. If bindings are not enabled, then no bound actions will be executed.
     * By default, bindings are not enabled.
     */
    void enableBindings();

    /**
     * Disable button bindings. If bindings are not enabled, then no bound actions will be executed.
     * By default, bindings are not enabled.
     */
    void disableBindings();

    /**
     * A physical control on a Gamepad, eg a thumbstick or directional pad.
     */
    interface Control {
        int getIdentifier(Axis axis);
    }

    /**
     * An axis for a control, eg horizontal, vertical.
     */
    interface Axis {
        int getIdentifier();
    }

    /**
     * A physical button on a gamepad.
     */
    interface Button {
        int getIdentifier();
    }

    /**
     * Types of button press actions.
     */
    enum PressType {
        PRESS,
        RELEASE,
        HOLD,
    }

    /**
     * A (Button, PressType) tuple for identifying button bindings.
     */
    class Binding {
        private final Set<Button> buttons;
        private final PressType pressType;

        /**
         * Construct a new Binding identifier.
         *
         * @param buttons   the buttons
         * @param pressType the press type
         */
        public Binding(final Collection<Button> buttons, final PressType pressType) {
            this.buttons = new HashSet<>(requireNonNull(buttons));
            this.pressType = requireNonNull(pressType);
        }

        /**
         * Construct a new Binding identifier.
         *
         * @param button    the button
         * @param pressType the press type
         */
        public Binding(final Button button, final PressType pressType) {
            this(Collections.singleton(button), pressType);
        }

        /**
         * Get the button.
         *
         * @return the button
         */
        public Set<Button> getButtons() {
            return buttons;
        }

        /**
         * Get the press type.
         *
         * @return the press type.
         */
        public PressType getPressType() {
            return pressType;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean equals(final Object o) {
            if (this == o) {
                return true;
            }
            if (o == null || getClass() != o.getClass()) {
                return false;
            }

            final Binding binding = (Binding) o;

            if (!buttons.equals(binding.buttons)) {
                return false;
            }
            if (pressType != binding.pressType) {
                return false;
            }

            return true;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public int hashCode() {
            int result = buttons.hashCode();
            result = 31 * result + pressType.hashCode();
            return result;
        }

        @Override
        public String toString() {
            return "Binding{"
                           + "buttons=" + buttons
                           + ", pressType=" + pressType
                           + '}';
        }
    }

    /**
     * A (Control, Axis) tuple for identifying mode mappings.
     */
    class Mapping {
        private final Control control;
        private final Axis axis;

        /**
         * Construct a new ModeIdentifier.
         *
         * @param control the control
         * @param axis    the axis
         */
        public Mapping(final Control control, final Axis axis) {
            this.control = requireNonNull(control);
            this.axis = requireNonNull(axis);
        }

        /**
         * Get the control.
         *
         * @return the control
         */
        public Control getControl() {
            return control;
        }

        /**
         * Get the axis.
         *
         * @return the axis
         */
        public Axis getAxis() {
            return axis;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean equals(final Object o) {
            if (this == o) {
                return true;
            }
            if (o == null || getClass() != o.getClass()) {
                return false;
            }

            final Mapping that = (Mapping) o;

            if (!axis.equals(that.axis)) {
                return false;
            }
            if (!control.equals(that.control)) {
                return false;
            }

            return true;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public int hashCode() {
            int result = control.hashCode();
            result = 31 * result + axis.hashCode();
            return result;
        }

        @Override
        public String toString() {
            return "Mapping{"
                           + "control=" + control
                           + ", axis=" + axis
                           + '}';
        }
    }
}
