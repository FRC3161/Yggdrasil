/*
 * Copyright (c) 2016-2017, FRC3161.
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

package ca.team3161.lib.robot.motion.drivetrains;

import static ca.team3161.lib.utils.Utils.normalizePwm;
import static java.util.Objects.requireNonNull;

import java.util.Arrays;
import java.util.Collection;

import ca.team3161.lib.utils.ComposedComponent;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class ArcadeDrivetrain extends AbstractDrivetrainBase implements ComposedComponent<SpeedControllerGroup> {

    private final SpeedControllerGroup leftControllers;
    private final SpeedControllerGroup rightControllers;
    private final DifferentialDrive drivebase;

    private volatile double forwardTarget;
    private volatile double turnTarget;

    private ArcadeDrivetrain(ArcadeDrivetrain.Builder builder) {
        this.leftControllers = builder.leftControllers;
        this.rightControllers = builder.rightControllers;
        this.drivebase = new DifferentialDrive(leftControllers, rightControllers);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void defineResources() {
        require(leftControllers);
        require(rightControllers);
        leftControllers.getAll().forEach(this::require);
        rightControllers.getAll().forEach(this::require);
    }

    public void setForwardTarget(double forwardTarget) {
        this.forwardTarget = normalizePwm(forwardTarget);
    }

    public void setTurnTarget(double turnTarget) {
        this.turnTarget = normalizePwm(turnTarget);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stop() {
        setForwardTarget(0);
        setTurnTarget(0);
        leftControllers.disable();
        rightControllers.disable();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void task() throws Exception {
        drivebase.arcadeDrive(forwardTarget, turnTarget);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Collection<SpeedControllerGroup> getComposedComponents() {
        return Arrays.asList(leftControllers, rightControllers);
    }

    /**
     * A Builder for arcade drivetrains. Provides an easy way to make arcade drivetrains without having to remember
     * parameter order.
     */
    public static class Builder {
        private SpeedControllerGroup leftControllers;
        private SpeedControllerGroup rightControllers;

        /**
         * Use the given parameters and construct a TankDrivetrain.
         *
         * @return a TankDrivetrain instance
         */
        public ArcadeDrivetrain build() {
            verify();
            return new ArcadeDrivetrain(this);
        }

        /**
         * Set the left-side SpeedControllerGroup to use. A group can contain one or more SpeedControllers.
         *
         * @param leftControllers the left controllers
         * @return this builder
         */
        public Builder leftControllers(SpeedControllerGroup leftControllers) {
            this.leftControllers = leftControllers;
            return this;
        }

        public Builder leftControllers(SpeedController... leftControllers) {
            return leftControllers(new SpeedControllerGroup(leftControllers));
        }

        /**
         * Set the right-side SpeedControllerGroup to use. A group can contain one or more SpeedControllers.
         *
         * @param rightControllers the right controllers
         * @return this builder
         */
        public Builder rightControllers(SpeedControllerGroup rightControllers) {
            this.rightControllers = rightControllers;
            return this;
        }

        public Builder rightControllers(SpeedController... rightControllers) {
            return rightControllers(new SpeedControllerGroup(rightControllers));
        }

        private void verify() {
            requireNonNull(leftControllers, error("leftControllers"));
            requireNonNull(rightControllers, error("rightControllers"));
        }

        private static String error(String component) {
            return component + " must not be null!";
        }

    }

}
