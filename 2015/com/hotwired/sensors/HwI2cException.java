package com.revAmped.sensors;

/**
 * Used for I2C Exception
 */
public class HwI2cException
    extends RuntimeException
{
    /**
     * Constructs a new {@code HwI2cException} with the current stack trace
     * and the specified detail message.
     *
     * @param detailMessage
     *            the detail message for this exception.
     */
    public HwI2cException (String detailMessage) {
        super(detailMessage);
    }

    /**
     * Constructs a new {@code HwI2cException} with the current stack trace
     * and the specified detail message.
     *
     * @param format
     *            the message format for this exception.
     * @param args message arguments
     */
    public HwI2cException (String format, Object... args) {
        super(String.format(format, args));
    }

    /**
     * Constructs a new {@code HwI2cException} with the current stack trace,
     * the specified detail message and the specified cause.
     *
     * @param detailMessage
     *            the detail message for this exception.
     * @param throwable
     *            the cause of this exception.
     */
    public HwI2cException(String detailMessage, Throwable throwable) {
        super(detailMessage, throwable);
    }

    /**
     * Constructs a new {@code HwI2cException} with the current stack trace
     * and the specified cause.
     *
     * @param throwable
     *            the cause of this exception.
     */
    public HwI2cException(Throwable throwable) {
        super(throwable);
    }
}
