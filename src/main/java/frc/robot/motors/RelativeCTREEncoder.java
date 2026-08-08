package frc.robot.motors;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Encoder wrapper around the internal Talon FX sensor.
 *
 * <p>Positions are reported in rotations. Phoenix 6 reports velocity in rotations per second,
 * so this wrapper converts to RPM for consistency with the {@link Encoder} interface.
 */
public class RelativeCTREEncoder implements Encoder {
    private static final double SECONDS_PER_MINUTE = 60.0;

    private final TalonFX motor;

    /**
     * Creates an encoder wrapper for a Talon FX internal sensor.
     *
     * @param motor Talon FX whose internal sensor should be exposed
     */
    public RelativeCTREEncoder(TalonFX motor) {
        this.motor = motor;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble() * SECONDS_PER_MINUTE;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setPosition(double position) {
        motor.setPosition(position);
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
    public TalonFX getRawEncoder() {
        return motor;
    }
}
