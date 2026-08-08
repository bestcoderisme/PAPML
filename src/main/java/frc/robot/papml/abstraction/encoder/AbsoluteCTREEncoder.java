package frc.robot.papml.abstraction.encoder;

import com.ctre.phoenix6.hardware.CANcoder;

/**
 * Encoder wrapper around the internal Talon FX sensor.
 *
 * <p>Positions are reported in rotations. Phoenix 6 reports velocity in rotations per second,
 * so this wrapper converts to RPM for consistency with the {@link Encoder} interface.
 */
public class AbsoluteCTREEncoder implements Encoder {
    private static final double SECONDS_PER_MINUTE = 60.0;

    private final CANcoder encoder;

    /**
     * Creates an encoder wrapper for a Talon FX internal sensor.
     *
     * @param motor Talon FX whose internal sensor should be exposed
     */
    public AbsoluteCTREEncoder(CANcoder encoder) {
        this.encoder = encoder;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getPosition() {
        return encoder.getPosition().getValueAsDouble();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getVelocity() {
        return encoder.getVelocity().getValueAsDouble() * SECONDS_PER_MINUTE;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setPosition(double position) {
        encoder.setPosition(position);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void zeroEncoder() {
        setPosition(0.0);
    }
    
     /**
     * {@inheritDoc}
     */
    @Override
    public CANcoder getRawEncoder() {
        return encoder;
    }
}
