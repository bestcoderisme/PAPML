package frc.robot.papml;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleFunction;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.papml.CharacterizationConstraints.RangeConstraints;
import frc.robot.papml.abstraction.motor.Motor;

public class VariableGravityPositionSubsystem extends SmartSubsystem {
    protected ArmFeedforward feedforward;
    protected CharacterizationRoutine routine;
    protected RangeConstraints rangeConstraints;
    ArmMotor motor;
    public VariableGravityPositionSubsystem(String name, Motor motor, double accuracyThreshold, int oscillationLimit, CharacterizationConstraints constraints, double gearRatio, double targetPositionForPIDTuning) {
        super(name, motor, accuracyThreshold, oscillationLimit);
        this.motor = new ArmMotor(motor, gearRatio);
        samples = new FFCharacterizationSamples(FFCharacterizationSamples.GravityMode.COSINE);
        FFConstants FFconstants = FFConstants.getFFFromPreferences(name);
        feedforward = new ArmFeedforward(FFconstants.kS, FFconstants.kG, FFconstants.kV, FFconstants.kA);
        this.routine = new CharacterizationRoutine(samples, constraints, motor);
        this.rangeConstraints = constraints.rangeConstraints;
        this.debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
        initializeSearchAlgorithms(targetPositionForPIDTuning);
    }

    public VariableGravityPositionSubsystem withOffsetFromPreferences(){
        motor.setOffsetRadians(Preferences.getDouble(name + "/Offset", 0));
        return this;
    }

    //for arms using absolute encoder, get value from the run into hard stop calibration command
    public void setOffsetRadians(double offset){
        motor.setOffsetRadians(offset);
    }

    private Command runIntoHardStopCalibration(double voltage, double stoppedVelocityThreshold, Runnable offsetSetter) {
        return Commands.runOnce(() -> {
            motor.getMotor().setVoltage(voltage);
        }).until(() -> Math.abs(motor.getMotor().getVelocity()) > stoppedVelocityThreshold)
        .andThen(Commands.waitUntil(() -> Math.abs(motor.getMotor().getVelocity()) < stoppedVelocityThreshold))
        .andThen(Commands.runOnce(() -> {
            offsetSetter.run();
            motor.getMotor().setVoltage(0);
            SmartDashboard.putNumber(name + "/Offset", motor.offsetRadians);
            Preferences.setDouble(name + "/Offset", motor.offsetRadians);
        }));
    }

    public Command runIntoHardStopCalibrationDegrees(double voltage, double stoppedVelocityThreshold, double degreesAtHardStop) {
        return runIntoHardStopCalibration(voltage, stoppedVelocityThreshold, () -> {
            motor.setOffsetFromCurrentDegrees(degreesAtHardStop);
        });
    }

    public Command runIntoHardStopCalibrationRadians(double voltage, double stoppedVelocityThreshold, double radiansAtHardStop) {
        return runIntoHardStopCalibration(voltage, stoppedVelocityThreshold, () -> {
            motor.setOffsetFromCurrentRadians(radiansAtHardStop);
        });
    }

    public Command runIntoHardStopCalibrationRotations(double voltage, double stoppedVelocityThreshold, double rotationsAtHardStop) {
        return runIntoHardStopCalibration(voltage, stoppedVelocityThreshold, () -> {
            motor.setOffsetFromCurrentRotations(rotationsAtHardStop);
        });
    }
    @Override
    protected SearchAlgorithm createSearchAlgorithm(DoubleConsumer setConstant, String constantName, double target){
        AtomicBoolean reachedCalibrationTarget = new AtomicBoolean(false);
        DoubleFunction<Command> test =(double constant) -> 
        Commands.sequence(
            Commands.runOnce(()->
            {
                setTargetPosition(rangeConstraints.minPosition); //set to minimum not zero
                reachedCalibrationTarget.set(false);
                resetOscillationTracking();
            }
            ),
            Commands.waitUntil(() -> Math.abs(motor.getAngleInRadians() - rangeConstraints.minPosition) < accuracyThreshold),
            Commands.runOnce(
                () -> {
                setConstant.accept(constant);
                setTargetPosition(target);
                timer.restart();
                debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
                }
            ),
            Commands.waitUntil(
                () -> {
                    if(!isWithinOscillationLimit()){
                        return true;
                    }
                    reachedCalibrationTarget.set(isPositionSettled());
                    return reachedCalibrationTarget.get();
                }
            ).withTimeout(3),
            Commands.runOnce(
                () -> {
                    lastTime = currentTime;
                    currentTime = reachedCalibrationTarget.get() ? timer.get() : 3;
                    setTargetPosition(rangeConstraints.minPosition); //set to minimum not zero
                    motor.getMotor().stop();
                }
            )
        );
        BooleanSupplier isValid = () ->  {return currentTime <= lastTime;};


        return new SearchAlgorithm(
            0.000005,
            0.00001,
            test,
            isValid,
            10.0,
            constantName
        );
    }

    @Override
    void setFFWithPreferences() {
        FFConstants coeffs = FFConstants.getFFFromPreferences(name);
        feedforward = new ArmFeedforward(coeffs.kS, coeffs.kG, coeffs.kV, coeffs.kA);
    }

    @Override
    Command calculateFFGains() {
        return routine.equilibriumRoutine(this, () -> motor.getAngleInRadians());
    }

    @Override
    double getVoltageFromPIDF() {
        return pid.calculate(motor.getAngleInRadians(), target) + feedforward.calculate(target, 0);
    }

    public Command regressSamples(){
        return Commands.runOnce(()->{
            regressor = new FFRegression(samples, false, true);
            FFConstants coeffs = regressor.getCoefficients();
            coeffs.publishToSmartDashboard(name);
            coeffs.publishToPreferences(name);
            setFFWithPreferences();
        });
    }

    private class ArmMotor {
        private final Motor motor;
        private final double gearRatio;
        private double offsetRadians;

        ArmMotor(Motor motor, double gearRatio){
            this.motor = motor;
            this.gearRatio = gearRatio;
        }

        public void setOffsetRadians(double offset){
            this.offsetRadians = offset;
        }

        public void setOffsetFromCurrentRadians(double currAngle){
            this.offsetRadians = currAngle - motor.getPosition() * 2 * Math.PI / gearRatio;
        }

        public void setOffsetFromCurrentRotations(double currAngle){
            this.setOffsetFromCurrentRadians(currAngle * 2 * Math.PI);
        }

        public void setOffsetFromCurrentDegrees(double currAngle){
            this.setOffsetFromCurrentRadians(currAngle/360*2*Math.PI);
        }

        public double getAngleInRadians(){
            return motor.getPosition() * 2 * Math.PI / gearRatio + offsetRadians;
        }

        public Motor getMotor(){
            return motor;
        }
    }

    public void setTargetPosition(double target) {
        this.target = target;
        SmartDashboard.putNumber(name + "/TargetPosition", target);
    }

    public Command setTargetPositionCmd(double target) {
        return Commands.runOnce(() -> {
            setTargetPosition(target);
        }, this);
    }

    protected boolean isPositionSettled() {     
        return debouncer.calculate(
            Math.abs(motor.getAngleInRadians() - target) < accuracyThreshold
        );
    }

    @Override
    protected void publishTelemetry() {
        super.publishTelemetry();
        routine.publishTelemetry(name);
        searchAlgorithmForkP.publishTelemetry(this);
        SmartDashboard.putNumber(name + "/CurrentPosition", motor.getAngleInRadians());
        SmartDashboard.putNumber(name + "/AutoTune/Samples", samples.getSamples().size());
    }

}
