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

package ca.team3161.lib.utils;

import java.util.Objects;

/**
 * Assertions. Contains static methods which take a boolean condition and throw
 * an exception if these conditions do not hold. Useful for debugging and during
 * development.
 */
public final class Assert {

    private Assert() {
    }

    /**
     * Assert that a condition is true.
     *
     * @param condition throw an exception if the given condition does not hold
     */
    public static void assertTrue(final boolean condition) {
        if (!condition) {
            throw new AssertionException();
        }
    }

    /**
     * Assert that a condition is true.
     *
     * @param reason    why the condition should hold
     * @param condition throw an exception if the given condition does not hold
     */
    public static void assertTrue(final String reason, final boolean condition) {
        Objects.requireNonNull(reason);
        if (!condition) {
            throw new AssertionException(reason);
        }
    }

    /**
     * Assert that a condition is false.
     *
     * @param condition throw an exception if the given condition holds
     */
    public static void assertFalse(final boolean condition) {
        assertTrue(!condition);
    }

    /**
     * Assert rhat a condition is false.
     *
     * @param reason    why the condition should not hold
     * @param condition throw an exception if the given condition holds
     */
    public static void assertFalse(final String reason, final boolean condition) {
        Objects.requireNonNull(reason);
        assertTrue(reason, !condition);
    }

    /**
     * Indicates that an Assert statement has failed. This is intended to cause
     * a crash and halt your robot code, leaving the cRIO/RoboRIO responsible for
     * stopping all motor controllers, relays, etc. If you ever find yourself
     * using a try/catch to catch an AssertionException, this is a very strong
     * indicator that you are using Assert incorrectly.
     */
    public static final class AssertionException extends RuntimeException {

        /**
         * Create a new AssertionException with generic cause.
         */
        private AssertionException() {
            super("Unspecified assertion failure");
        }

        /**
         * Create a new AssertionException.
         *
         * @param cause the cause for this exception to have occurred
         */
        private AssertionException(final String cause) {
            super(cause);
        }

    }

}
