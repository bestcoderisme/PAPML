package frc.robot.motors;

/**
 * Common abstraction for motor controller wrappers.
 *
 * <p>All velocities are expressed in RPM. All positions are expressed in rotations.
 */
public interface Motor {
    /**
     * Sets open-loop output as a fraction of full output.
     *
     * @param percentOutput output from -1.0 to 1.0
     */
    void set(double percentOutput);

    /**
     * Sets output voltage directly.
     *
     * @param volts desired output voltage
     */
    void setVoltage(double volts);

    /**
     * Sets the closed-loop velocity target in RPM.
     *
     * @param rpm desired velocity in RPM
     */
    // void setVelocity(double rpm);

    /**
     * Sets the closed-loop position target in rotations.
     *
     * @param position desired position in rotations
     */
    // void setPosition(double position);

    /**
     * Stops the motor output.
     */
    void stop();

    /**
     * Returns the measured velocity in RPM.
     *
     * @return measured velocity in RPM
     */
    double getVelocity();

    /**
     * Returns the measured position in rotations.
     *
     * @return measured position in rotations
     */
    double getPosition();

    /**
     * Returns the measured motor current in amps.
     *
     * @return measured current in amps
     */
    double getCurrent();

    /**
     * Returns the measured motor temperature in degrees Celsius.
     *
     * @return measured motor temperature in degrees Celsius
     */
    double getTemperature();

    /**
     * Sets whether positive output should be inverted.
     *
     * @param inverted true to invert positive output direction
     */
    void setInverted(boolean inverted);

    /**
     * Sets brake or coast neutral behavior.
     *
     * @param brake true for brake mode, false for coast mode
     */
    void setBrakeMode(boolean brake);

    /**
     * Sets the motor current limit in amps.
     *
     * @param amps current limit in amps
     */
    // void setCurrentLimit(int amps);

    /**
     * Sets the time required to ramp from zero to full output.
     *
     * @param secondsToFull ramp time in seconds
     */
    // void setRampRate(double secondsToFull);

    /**
     * Configures this motor to follow another motor when supported by the implementation.
     *
     * @param leader leader motor to follow
     * @param invert true to invert the follower's output relative to the leader, false for same direction
     */
    void follow(Motor leader, boolean invert);

    /**
     * Returns whether the motor controller appears connected.
     *
     * @return true if the controller appears connected
     */
    boolean isConnected();

    /**
     * Returns the encoder associated with this motor.
     *
     * @return associated encoder
     */
    Encoder getEncoder();

    /**
     * Returns the raw motor controller object used by the underlying vendor library.
     *
     * @return raw motor controller object
     */
    Object getRawMotor();
}
