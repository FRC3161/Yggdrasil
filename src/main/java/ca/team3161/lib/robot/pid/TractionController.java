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

package ca.team3161.lib.robot.pid;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.pidwrappers.PIDEncoder;

/**
 * A VelocityController with toggleable "traction control". When disabled, the PID control is bypassed and the input
 * values are fed directly into the backing SpeedController instance.
 */
public class TractionController extends VelocityController {

    private boolean tractionEnabled;

    public TractionController(final SpeedController speedController, final PIDEncoder encoder, final float maxRotationalRate, final float kP, final float kI, final float kD, final float maxIntegralError, final float deadband) {
        super(speedController, encoder, maxRotationalRate, kP, kI, kD, maxIntegralError, deadband);
    }

    public TractionController(final SpeedController speedController, final PIDRateValueSrc<PIDEncoder> encoderPidSrc, final float maxRotationalRate, final float kP, final float kI, final float kD, final float maxIntegralError, final float deadband) {
        super(speedController, encoderPidSrc, maxRotationalRate, kP, kI, kD, maxIntegralError, deadband);
    }

    /**
     * Enable traction control
     * @param tractionEnabled whether to enable traction control
     */
    public void setTractionEnabled(boolean tractionEnabled) {
        this.tractionEnabled = tractionEnabled;
    }

    /**
     * Check if traction control is enabled
     * @return the current traction control state
     */
    public boolean isTractionEnabled() {
        return tractionEnabled;
    }

    @Override
    public void set(final double v) {
        if (tractionEnabled) {
            super.set(v);
        } else {
            speedController.set(v);
        }
    }
}
