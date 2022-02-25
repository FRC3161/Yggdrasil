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

import static ca.team3161.lib.utils.Utils.normalizePwm;
import static java.util.Objects.requireNonNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ca.team3161.lib.utils.ComposedComponent;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * A prepackaged mecanum drive solution, suitable for 4-wheeled Mecanum drivetrains.
 * See {@link ca.team3161.lib.robot.pid.VelocityController} for a recommended SpeedController to use for each wheel.
 */
public class MecanumDrivetrain extends AbstractDrivetrainBase implements ComposedComponent<Object> {

    private final SpeedController frontLeftController;
    private final SpeedController frontRightController;
    private final SpeedController backLeftController;
    private final SpeedController backRightController;
    private final Optional<Gyro> gyro;
    private final MecanumDrive drivebase;

    private volatile double forwardTarget = 0;
    private volatile double strafeTarget = 0;
    private volatile double rotateTarget = 0;

    private MecanumDrivetrain(Builder builder) {
        this.frontLeftController = builder.frontLeftController;
        this.frontRightController = builder.frontRightController;
        this.backLeftController = builder.backLeftController;
        this.backRightController = builder.backRightController;
        this.gyro = Optional.ofNullable(builder.gyro);
        this.drivebase = new MecanumDrive(frontLeftController, backLeftController, frontRightController, backRightController);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void defineResources() {
        require(frontLeftController);
        require(frontRightController);
        require(backLeftController);
        require(backRightController);
        gyro.ifPresent(this::require);
    }

    /**
     * Set the "forward target", which is a value in the range [-1.0, 1.0]; 1.0 representing full speed forward,
     * -1.0 representing full speed backward, and 0 representing no forward or backward movement.
     * @param forwardTarget the forward target
     */
    public void setForwardTarget(double forwardTarget) {
        this.forwardTarget = normalizePwm(forwardTarget);
    }

    /**
     * Set the "strafe target", which is a value in the range [-1.0, 1.0]; 1.0 representing full speed right,
     * -1.0 representing full speed left, and 0 representing no strafing.
     * @param strafeTarget the forward target
     */
    public void setStrafeTarget(double strafeTarget) {
        this.strafeTarget = normalizePwm(strafeTarget);
    }

    /**
     * Set the "rotate target", which is a value in the range [-1.0, 1.0]; 1.0 representing full speed clockwise,
     * -1.0 representing full speed counterclockwise, and 0 representing no rotation.
     * @param rotateTarget the forward target
     */
    public void setRotateTarget(double rotateTarget) {
        this.rotateTarget = normalizePwm(rotateTarget);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stop() {
        setForwardTarget(0);
        setStrafeTarget(0);
        setRotateTarget(0);
        frontLeftController.disable();
        frontRightController.disable();
        backLeftController.disable();
        backRightController.disable();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void task() throws Exception {
        drivebase.driveCartesian(strafeTarget, forwardTarget, rotateTarget,
                gyro.flatMap(g -> Optional.ofNullable(g.getAngle())).orElse(0.0));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Collection<Object> getComposedComponents() {
        List<Object> components = new ArrayList<>();
        components.addAll(Arrays.asList(frontLeftController, frontRightController, backLeftController, backRightController));
        gyro.ifPresent(components::add);
        return components;
    }

    /**
     * A Builder for mecanum drivetrains. Provides an easy way to make mecanum drivetrains without having to remember
     * parameter order.
     */
    public static class Builder {
        private SpeedController frontLeftController;
        private SpeedController frontRightController;
        private SpeedController backLeftController;
        private SpeedController backRightController;
        private Gyro gyro;

        /**
         * Use the given parameters and construct a MecanumDrivetrain.
         * @return a MecanumDrivetrain instance
         */
        public MecanumDrivetrain build() {
            verify();
            return new MecanumDrivetrain(this);
        }

        /**
         * Set the front left controller to use.
         * @param frontLeftController the controller
         * @return this builder
         */
        public Builder frontLeftController(SpeedController frontLeftController) {
            this.frontLeftController = frontLeftController;
            return this;
        }

        /**
         * Set the front right controller to use.
         * @param frontRightController the controller
         * @return this builder
         */
        public Builder frontRightController(SpeedController frontRightController) {
            this.frontRightController = frontRightController;
            return this;
        }

        /**
         * Set the back left controller to use.
         * @param backLeftController the controller
         * @return this builder
         */
        public Builder backLeftController(SpeedController backLeftController) {
            this.backLeftController = backLeftController;
            return this;
        }

        /**
         * Set the back right controller to use.
         * @param backRightController the controller
         * @return this builder
         */
        public Builder backRightController(SpeedController backRightController) {
            this.backRightController = backRightController;
            return this;
        }

        /**
         * Set the gyro to use (optional).
         * @param gyro the gyro
         * @return this builder
         */
        public Builder gyro(Gyro gyro) {
            this.gyro = gyro;
            return this;
        }

        private void verify() {
            requireNonNull(frontLeftController, error("frontLeftController"));
            requireNonNull(frontRightController, error("frontRightController"));
            requireNonNull(backLeftController, error("backLeftController"));
            requireNonNull(backRightController, error("backRightController"));
        }

        private static String error(String component) {
            return component + " must not be null!";
        }

    }

}
