package frc.robot.motors;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * {@link Motor} implementation backed by a Phoenix 6 {@link TalonFX}.
 *
 * <p>All externally visible velocities use RPM. Phoenix 6 closed-loop velocity requests use
 * rotations per second internally, so this implementation converts between the two.
 */
public class CTREMotor implements Motor {
    private static final double SECONDS_PER_MINUTE = 60.0;

    private final TalonFX motor;
    private final Encoder encoder;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final PositionVoltage positionRequest = new PositionVoltage(0.0);
    private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    private final OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    private final ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();

    /**
     * Creates a Talon FX motor wrapper.
     *
     * @param motor the vendor TalonFX object to wrap
     * @param encoder encoder abstraction associated with this motor; if null, the primary Talon FX encoder wrapper is used
     */
    public CTREMotor(TalonFX motor, Encoder encoder) {
        this.motor = motor;
        if(encoder == null) {
            this.encoder = new RelativeCTREEncoder(motor);
        } else {
            this.encoder = encoder;
        }
    }

    /**
     * Creates a Talon FX motor wrapper.
     *
     * @param motor the vendor TalonFX object to wrap
     */
    public CTREMotor(TalonFX motor) {
        this.motor = motor;
        this.encoder = new RelativeCTREEncoder(motor);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void set(double percentOutput) {
        motor.setControl(dutyCycleRequest.withOutput(percentOutput));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    /**
     * {@inheritDoc}
     */
    // @Override
    // public void setVelocity(double rpm) {
    //     motor.setControl(velocityRequest.withVelocity(rpm / SECONDS_PER_MINUTE));
    // }

    /**
     * {@inheritDoc}
     */
    // @Override
    // public void setPosition(double position) {
    //     motor.setControl(positionRequest.withPosition(position));
    // }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stop() {
        motor.setControl(dutyCycleRequest.withOutput(0.0));
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
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getTemperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setInverted(boolean inverted) {
        motorOutputConfigs.Inverted =
                inverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(motorOutputConfigs);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setBrakeMode(boolean brake) {
        motorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.getConfigurator().apply(motorOutputConfigs);
    }

    /**
     * {@inheritDoc}
     */
    // @Override
    // public void setCurrentLimit(int amps) {
    //     currentLimitsConfigs.SupplyCurrentLimit = amps;
    //     currentLimitsConfigs.SupplyCurrentLowerLimit = amps;
    //     currentLimitsConfigs.SupplyCurrentLimitEnable = amps > 0;
    //     currentLimitsConfigs.SupplyCurrentLowerTime = 0.0;
    //     motor.getConfigurator().apply(currentLimitsConfigs);
    // }

    // /**
    //  * {@inheritDoc}
    //  */
    // @Override
    // public void setRampRate(double secondsToFull) {
    //     openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = secondsToFull;
    //     openLoopRampsConfigs.VoltageOpenLoopRampPeriod = secondsToFull;
    //     closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = secondsToFull;
    //     closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = secondsToFull;
    //     motor.getConfigurator().apply(openLoopRampsConfigs);
    //     motor.getConfigurator().apply(closedLoopRampsConfigs);
    // }

    /**
     * {@inheritDoc}
     */
    @Override
    public void follow(Motor leader, boolean invert) {
        if (leader instanceof CTREMotor ctreLeader) {
            motor.setControl(
                    new Follower(ctreLeader.motor.getDeviceID(), invert ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean isConnected() {
        return motor.isConnected();
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
    public TalonFX getRawMotor() {
        return motor;
    }
}
