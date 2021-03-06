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

import ca.team3161.lib.robot.utils.ChassisParameters;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * A Position Estimator for plain Skid Steer drivetrains, eg standard tank and arcade drive.
 */
public class SkidSteerPositionEstimator extends AbstractPositionEstimator {

    /**
     * Construct a new SkidSteerPositionEstimator.
     *
     * @param chassisParameters a physical description of the robot
     * @param accelerometer     an accelerometer
     * @param gyro              a gyroscope
     * @param frontLeftEncoder  the encoder attached to the output of the front left wheel
     * @param frontRightEncoder the encoder attached to the output of the front right wheel
     * @param backLeftEncoder   the encoder attached to the output of the back left wheel
     * @param backRightEncoder  the encoder attached to the output of the back right wheel
     */
    public SkidSteerPositionEstimator(final ChassisParameters chassisParameters,
                                      final Accelerometer accelerometer, final Gyro gyro,
                                      final Encoder frontLeftEncoder, final Encoder frontRightEncoder,
                                      final Encoder backLeftEncoder, final Encoder backRightEncoder) {
        super(chassisParameters, accelerometer, gyro, frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder);
    }

    @Override
    protected void updateSteerSpecificParameters() {
        w1 = frontRightEncoder.getRate();
        w2 = frontLeftEncoder.getRate();

        vx = (w1 + w2) / 2;
        vy = 0;
        dw = (w2 - w1) / 2 / chassisParameters.getWheelBaseWidth();
    }
}
