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

package ca.team3161.lib.robot.pid;

import static ca.team3161.lib.robot.pid.PIDUtils.validate;
import static java.util.Objects.requireNonNull;

import java.util.Collection;
import java.util.Collections;

import ca.team3161.lib.utils.ComposedComponent;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.pidwrappers.PIDEncoder;

/**
 * A PID source backed by a physical Encoder. Defaults to returning encoder rate values.
 */
public class EncoderPIDSrc implements PIDRateValueSrc<PIDEncoder>, PIDRawValueSrc<PIDEncoder>, ComposedComponent<Encoder> {

    private final PIDEncoder enc;
    private PIDSourceType sourceType = PIDSourceType.kRate;

    /**
     * Create a new EncoderPidSrc instance.
     *
     * @param enc an Encoder object to use as a PIDSrc
     */
    public EncoderPIDSrc(final PIDEncoder enc) {
        this.enc = requireNonNull(enc);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public PIDEncoder getSensor() {
        return enc;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Float getPIDValue() {
        switch (sourceType) {
            case kRate:
                return (float) enc.getRate();
            case kDisplacement:
                return (float) enc.get();
            default:
                throw new InvalidPIDSourceTypeException(sourceType);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Collection<Encoder> getComposedComponents() {
        return Collections.singleton(enc);
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return sourceType;
    }

    @Override
    public void setPIDSourceType(final PIDSourceType pidSourceType) {
        this.sourceType = validate(pidSourceType);
    }
}
