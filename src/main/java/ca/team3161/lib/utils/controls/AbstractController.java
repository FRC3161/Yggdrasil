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

package ca.team3161.lib.utils.controls;

import static ca.team3161.lib.utils.Utils.requireNonNegative;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * A Gamepad which allows button bindings and control modes.
 */
public abstract class AbstractController extends RepeatingIndependentSubsystem implements Gamepad, LifecycleListener {

    /* The actual FIRST-provided input device that we are implementing a
    * convenience wrapper around.
    */
    protected final GenericHID backingHID;
    protected final Map<Mapping, Function<Double, Double>> controlsModeMap = new HashMap<>();
    protected final Map<Mapping, Consumer<Double>> controlsMapping = new HashMap<>();
    protected final Map<Binding, Runnable> buttonBindings = new ConcurrentHashMap<>();
    protected final Map<Button, Boolean> buttonStates = new ConcurrentHashMap<>();
    protected final int port;
    protected static final BitSet BOUND_PORTS = new BitSet();

    protected AbstractController(final int port, final long timeout, final TimeUnit timeUnit) {
        super(timeout, timeUnit);
        this.port = requireNonNegative(port);
        synchronized (BOUND_PORTS) {
            if (BOUND_PORTS.get(port)) {
                throw new IllegalStateException("Port " + port + " is already bound; cannot bind two input devices to the same port");
            }
            BOUND_PORTS.set(port);
        }
        backingHID = new Joystick(port); // Joystick happens to work well here, but any GenericHID should be fine
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

    /**
     * {@inheritDoc}
     */
    @Override
    public void defineResources() {
        // none!
    }

    /**
     * Get the set of Buttons on this controller.
     *
     * @return a set of Buttons
     */
    protected abstract Set<Button> getButtons();

    @Override
    public void task() throws Exception {
        final Map<Button, Boolean> previousButtonStates = new HashMap<>(buttonStates);
        getButtons().forEach(button -> buttonStates.put(button, getButton(button)));
        synchronized (controlsMapping) {
            controlsMapping.entrySet().forEach(mapping ->
                mapping.getValue().accept(getValue(mapping.getKey().getControl(), mapping.getKey().getAxis())));
        }
        synchronized (buttonBindings) {
            buttonBindings.entrySet().forEach((Map.Entry<Binding, Runnable> binding) -> {
                final Set<Button> buttons = binding.getKey().getButtons();
                final PressType pressType = binding.getKey().getPressType();
                final Runnable action = binding.getValue();

                final boolean currentlyHeld = buttons.stream().allMatch(buttonStates::get);
                final boolean previouslyHeld = buttons.stream().allMatch(previousButtonStates::get);
                switch (pressType) {
                    case PRESS:
                        if (currentlyHeld && !previouslyHeld) {
                            getExecutorService().submit(action);
                        }
                        break;
                    case RELEASE:
                        if (!currentlyHeld && previouslyHeld) {
                            getExecutorService().submit(action);
                        }
                        break;
                    case HOLD:
                        if (currentlyHeld) {
                            getExecutorService().submit(action);
                        }
                        break;
                    default:
                        throw new IllegalArgumentException("Gamepad on port " + Integer.toString(getPort())
                                                                   + " has binding for unknown button press type " + pressType);
                }
            });
        }
    }

    @Override
    public void lifecycleStatusChanged(final LifecycleEvent previous, final LifecycleEvent current) {
        switch (current) {
            case NONE:
            case ON_INIT:
            case ON_DISABLED:
            case ON_AUTO:
                disableBindings();
                break;
            case ON_TELEOP:
            case ON_TEST:
                enableBindings();
                break;
        }
    }

}
