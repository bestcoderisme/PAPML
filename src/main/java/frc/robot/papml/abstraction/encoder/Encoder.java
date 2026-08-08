package frc.robot.papml.abstraction.encoder;

/**
 * Common abstraction for rotary encoder feedback used by motor wrappers.
 *
 * <p>All positions are reported in rotations. All velocities are reported in RPM.
 */
public interface Encoder {
    /**
     * Returns the current encoder position in rotations.
     *
     * @return encoder position in rotations
     */
    double getPosition();

    /**
     * Returns the current encoder velocity in RPM.
     *
     * @return encoder velocity in RPM
     */
    double getVelocity();

    /**
     * Sets the encoder position in rotations.
     *
     * @param position new encoder position in rotations
     */
    void setPosition(double position);

    /**
     * Zeros the encoder position.
     */
    void zeroEncoder();

    /**
     * Returns the raw encoder object used by the underlying vendor library.
     * @return raw encoder object
      */
    Object getRawEncoder();
}
