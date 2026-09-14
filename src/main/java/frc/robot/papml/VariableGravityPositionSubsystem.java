package frc.robot.papml;

import java.util.function.DoubleConsumer;
import java.util.prefs.Preferences;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.papml.abstraction.motor.Motor;

public class VariableGravityPositionSubsystem extends SmartSubsystem {
    protected ArmFeedforward feedforward;
    protected CharacterizationRoutine routine;
    ArmMotor motor;
    public VariableGravityPositionSubsystem(String name, Motor motor, double accuracyThreshold, int oscillationLimit, CharacterizationConstraints constraints, double gearRatio) {
        super(name, motor, accuracyThreshold, oscillationLimit);
        this.motor = new ArmMotor(motor, gearRatio);
        samples = new FFCharacterizationSamples(FFCharacterizationSamples.GravityMode.COSINE);
        FFConstants FFconstants = FFConstants.getFFFromPreferences(name);
        feedforward = new ArmFeedforward(FFconstants.kS, FFconstants.kG, FFconstants.kV, FFconstants.kA);
        this.routine = new CharacterizationRoutine(samples, constraints, motor);

        this.debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
        initializeSearchAlgorithms();
    }

    //for arms using absolute encoder, get value from the run into hard stop calibration command
    public void setOffset(double offset){
        motor.setOffset(offset);
    }

    private Command runIntoHardStopCalibration(double voltage, double stoppedVelocityThreshold, Runnable offsetSetter) {
        return Commands.runOnce(() -> {
            motor.getMotor().setVoltage(voltage);
        }).until(() -> Math.abs(motor.getMotor().getVelocity()) > stoppedVelocityThreshold)
        .andThen(Commands.waitUntil(() -> Math.abs(motor.getMotor().getVelocity()) < stoppedVelocityThreshold))
        .andThen(Commands.runOnce(() -> {
            offsetSetter.run();
            motor.getMotor().setVoltage(0);
            SmartDashboard.putNumber(name + "/Offset", motor.offset);
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

    // @Override
    // protected double calculateFeedforward(double targetVelocity) {
    //     return feedforward.calculate(targetVelocity) + gravityCompensation;
    // }
    // @Override
    // protected SearchAlgorithm createSearchAlgorithm(DoubleConsumer setConstant, String constantName) {

    //     }

    @Override
    void setFFWithPreferences() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFFWithPreferences'");
    }

    @Override
    Command calculateFFGains() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'calculateFFGains'");
    }

    @Override
    double getVoltageFromPIDF() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVoltageFromPIDF'");
    }

    @Override
    SearchAlgorithm createSearchAlgorithm(DoubleConsumer setConstant, String constantName) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'createSearchAlgorithm'");
    }
    

    private class ArmMotor {
        private Motor motor;
        private double gearRatio;
        private double offset;

        ArmMotor(Motor motor, double gearRatio){
            this.motor = motor;
            this.gearRatio = gearRatio;
        }

        public void setOffset(double offset){
            this.offset = offset;
        }

        public void setOffsetFromCurrentRotations(double currAngle){
            this.offset = currAngle - motor.getPosition() / gearRatio;
        }

        public void setOffsetFromCurrentDegrees(double currAngle){
            this.offset = currAngle/360 - motor.getPosition() / gearRatio;
        }

        public void setOffsetFromCurrentRadians(double currAngle){
            this.offset = currAngle/(2*Math.PI) - motor.getPosition() / gearRatio;
        }

        public double getAngleInRotations(){
            return motor.getPosition() / gearRatio + offset;
        }

        public double getAngleInRadians(){
            return getAngleInRotations() * 2 * Math.PI;
        }

        public double getAngleInDegrees(){
            return getAngleInRotations() * 360;
        }

        public Motor getMotor(){
            return motor;
        }
    }
}
