package frc.robot.motors;

import com.revrobotics.RelativeEncoder;

/**
 * Encoder wrapper around a REV {@link RelativeEncoder}.
 *
 * <p>The REV relative encoder reports position in rotations and velocity in RPM by default, so
 * this wrapper exposes those units directly.
 */
public class RelativeREVEncoder implements Encoder {
    private final RelativeEncoder encoder;

    /**
     * Creates an encoder wrapper for the primary encoder on a SPARK controller.
     *
     * @param motor SPARK controller providing the encoder
     */
    public RelativeREVEncoder(RelativeEncoder encoder) {
        this.encoder = encoder;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getVelocity() {
        
        return encoder.getVelocity();
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
    public RelativeEncoder getRawEncoder() {
        return encoder;
    }
}