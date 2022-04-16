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

package ca.team3161.lib.robot.motion.tracking;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.util.Objects.requireNonNull;

import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.robot.utils.ChassisParameters;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import java.util.concurrent.TimeUnit;

/**
 * Abstract parent for drivetrain-specific classes which integrate accelerometer, gyroscope, and encoder data
 * in order to estimate total x, y distance travelled and degrees rotated by a robot.
 */
public abstract class AbstractPositionEstimator extends RepeatingPooledSubsystem {

    /**
     * Acceleration due to gravity, in meters per second.
     */
    public static final double GRAVITATIONAL_ACCELERATION = 9.80665;
    /* global co-ordinates */
    protected double x, y, theta, theta0;
    /* robot co-ordinates */
    protected double dx, dy, dtheta;
    /* previous co-ordinates */
    protected double px, py, ptheta, pAx, pAy;
    /* predictions */
    protected double mudx, mudy, mudtheta;

    protected long time, pt, dt;
    protected double vx, vy, dw;
    protected double e1, e2, e3;

    protected final ChassisParameters chassisParameters;
    protected final Accelerometer accelerometer;
    protected final Gyro gyro;
    protected final Encoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    protected double w1, w2, w3, w4;

    /**
     * Construct a new PositionEstimator. Requires physical parameters of the robot and a collection of sensors from which
     * to pull various data.
     *
     * @param chassisParameters physical parameters of the robot
     * @param accelerometer     an accelerometer to measure acceleration
     * @param gyro              a gyroscope to measure rotation
     * @param frontLeftEncoder  an encoder to measure movement of the front left wheel
     * @param frontRightEncoder an encoder to measure movement of the front right wheel
     * @param backLeftEncoder   an encoder to measure movement of the back left wheel
     * @param backRightEncoder  an encoder to measure movement of the back right wheel
     */
    public AbstractPositionEstimator(final ChassisParameters chassisParameters,
                                     final Accelerometer accelerometer, final Gyro gyro,
                                     final Encoder frontLeftEncoder, final Encoder frontRightEncoder,
                                     final Encoder backLeftEncoder, final Encoder backRightEncoder) {
        super(100, TimeUnit.MILLISECONDS);
        this.chassisParameters = requireNonNull(chassisParameters);
        this.accelerometer = requireNonNull(accelerometer);
        this.gyro = requireNonNull(gyro);
        this.frontLeftEncoder = requireNonNull(frontLeftEncoder);
        this.frontRightEncoder = requireNonNull(frontRightEncoder);
        this.backLeftEncoder = requireNonNull(backLeftEncoder);
        this.backRightEncoder = requireNonNull(backRightEncoder);

        final double distancePerPulse = 2 * Math.PI * chassisParameters.getWheelRadius() / chassisParameters.getEncoderCPR();
        frontLeftEncoder.setDistancePerPulse(distancePerPulse);
        frontRightEncoder.setDistancePerPulse(distancePerPulse);
        backLeftEncoder.setDistancePerPulse(distancePerPulse);
        backRightEncoder.setDistancePerPulse(distancePerPulse);
    }

    /**
     * Set the initial rotation of the robot. If the robot's initial heading is not to be interpreted as an angle of 0,
     * use this method to set the correct offset (for example, if your robot begins matches rotated).
     *
     * @param theta0 the initial angle
     */
    public void setInitialTheta(final double theta0) {
        this.theta0 = theta0;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void defineResources() {
        require(accelerometer);
        require(gyro);
        require(frontLeftEncoder);
        require(frontRightEncoder);
        require(backLeftEncoder);
        require(backRightEncoder);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void task() throws Exception {
        updateTime();
        updateSteerSpecificParameters();
        updateEstimate();
    }

    protected abstract void updateSteerSpecificParameters();

    protected void updateTime() {
        time = System.currentTimeMillis();
        dt = time - pt;
        pt = time;
    }

    protected void updateEstimate() {
        mudtheta = dw;
        mudx = vx;
        mudy = vy;

        pAx = pAx + accelerometer.getX() * dt * GRAVITATIONAL_ACCELERATION;
        pAy = pAy + accelerometer.getY() * dt * GRAVITATIONAL_ACCELERATION;

        e1 = pAx - mudx;
        e2 = pAy - mudy;
        e3 = gyro.getRate() - mudtheta;

        dx = mudx + 0.1 * e1;
        dy = mudy + 0.1 * e2;
        dtheta = mudtheta + 0.5 * e3;

        theta = ptheta + dtheta * dt;
        x = px + dx * dt * cos(theta) + dy * dt * sin(theta);
        y = py + dx * dt * sin(theta) + dy * dt * cos(theta);
    }

    public void reset() {
        x = 0;
        y = 0;
        theta = 0;
        theta0 = 0;
        dx = 0;
        dy = 0;
        dtheta = 0;
        px = 0;
        py = 0;
        ptheta = 0;
        pAx = 0;
        pAy = 0;
        mudx = 0;
        mudy = 0;
        mudtheta = 0;
        time = 0;
        pt = 0;
        dt = 0;
        vx = 0;
        vy = 0;
        dw = 0;
        e1 = 0;
        e2 = 0;
        e3 = 0;
        w1 = 0;
        w2 = 0;
        w3 = 0;
        w4 = 0;
    }

    /**
     * Get the current estimate of the robot's on-field position.
     *
     * @return the position estimate
     */
    public Position getEstimate() {
        return new Position(x, y, theta);
    }
}
