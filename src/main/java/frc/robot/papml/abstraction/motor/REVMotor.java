package frc.robot.papml.abstraction.motor;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.papml.abstraction.encoder.Encoder;
import frc.robot.papml.abstraction.encoder.RelativeREVEncoder;

/**
 * {@link Motor} implementation backed by a REV {@link SparkBase}.
 *
 * <p>This wrapper is intentionally null-safe. If constructed with a null motor, setter methods
 * become no-ops and getter methods return neutral values.
 */
public class REVMotor implements Motor {
    private final SparkBase motor;
    private final Encoder encoder;
    private final SparkBaseConfig config;

    /**
     * Creates a REV motor wrapper for either a SPARK MAX or SPARK Flex controller.
     *
     * @param motor backing SPARK controller
     * @param encoder encoder abstraction associated with this motor; if null, the primary REV
     *     relative encoder wrapper is used
     */
    public REVMotor(SparkBase motor, Encoder encoder) {
        this.motor = motor;
        this.encoder = encoder;
        this.config = createConfig(motor);
    }
    /**
     * Creates a REV motor wrapper for either a SPARK MAX or SPARK Flex controller.
     *
     * @param motor backing SPARK controller
     */
    public REVMotor(SparkBase motor) {
        this.motor = motor;
        this.encoder = new RelativeREVEncoder(motor.getEncoder());
        this.config = createConfig(motor);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void set(double percentOutput) {
            motor.set(percentOutput);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setVoltage(double volts) {
            motor.setVoltage(volts);
    }

    /**
     * {@inheritDoc}
     */
    // @Override
    // public void setVelocity(double rpm) {
    //     SparkClosedLoopController controller = getClosedLoopController();
    //         controller.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
    // }

    /**
     * {@inheritDoc}
     */
    // @Override
    // public void setPosition(double position) {
    //     SparkClosedLoopController controller = getClosedLoopController();
    //         controller.setSetpoint(position, SparkBase.ControlType.kPosition);
    // }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stop() {
            motor.stopMotor();
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
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getTemperature() {
        return motor.getMotorTemperature();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setInverted(boolean inverted) {
        config.inverted(inverted);
        applyConfig();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setBrakeMode(boolean brake) {
        config.idleMode(
                brake
                        ? SparkBaseConfig.IdleMode.kBrake
                        : SparkBaseConfig.IdleMode.kCoast);
        applyConfig();
    }

    /**
     * {@inheritDoc}
     */
    // @Override
    // public void setCurrentLimit(int amps) {
    //     config.smartCurrentLimit(amps);
    //     applyConfig();
    // }

    // /**
    //  * {@inheritDoc}
    //  */
    // @Override
    // public void setRampRate(double secondsToFull) {
    //     config.openLoopRampRate(secondsToFull);
    //     config.closedLoopRampRate(secondsToFull);
    //     applyConfig();
    // }

    /**
     * {@inheritDoc}
     */
    @Override
    public void follow(Motor leader, boolean invert) {
        if (leader instanceof REVMotor revLeader) {
            config.follow(revLeader.motor, invert);
            applyConfig();
        }
        else{
            throw new IllegalArgumentException("Leader must be a REVMotor to follow a REVMotor");
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean isConnected() {
        if (motor == null) {
            return false;
        }

        try {
            SparkBase.Faults faults = motor.getFaults();
            return motor.getLastError() != REVLibError.kCANDisconnected
                    && (faults == null || !faults.can);
        } catch (RuntimeException ex) {
            return false;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Encoder getEncoder() {
        return encoder;
    }
    
     /**
     * {@inheritDoc}
     */    
    @Override
    public Object getRawMotor() {
        return motor;
    }

    // Helper method to get the closed-loop controller, throwing an exception if the motor is null
    // private SparkClosedLoopController getClosedLoopController() {
    //     return motor.getClosedLoopController();
    // }

    private void applyConfig() {
        motor.configure(
                config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    private static SparkBaseConfig createConfig(SparkBase motor) {
        if (motor instanceof SparkMax) {
            return new SparkMaxConfig();
        }
        if (motor instanceof SparkFlex) {
            return new SparkFlexConfig();
        }
        return null;
    }
}
