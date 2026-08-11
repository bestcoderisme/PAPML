package frc.robot.papml;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import edu.wpi.first.wpilibj.Preferences;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SearchAlgorithm {
    private double low;
    private double high;
    private double precision;
    private final DoubleFunction<Command> test;
    private final BooleanSupplier isValid;
    private double max;
    private double start;
    private String itemBeingCharacterized;

    public SearchAlgorithm(double precision, double start, DoubleFunction<Command> test, BooleanSupplier isValid, double max, String itemBeingCharacterized) {
        this.low = start;
        this.high = start;
        this.precision = precision;
        this.test = test;
        this.isValid = isValid;
        this.max = max;
        this.start = start;
        this.itemBeingCharacterized = itemBeingCharacterized;
    }

    public void reset() {
        this.low = start;
        this.high = start;
    }

    public double getHigh() {
        return high;
    }

    public double getLow() {
        return low;
    }

    public double getMidpoint() {
        return (low + high) / 2.0;
    }

    private void tooHigh() {
        high = getMidpoint();
    }

    private void tooLow() {
        low = getMidpoint();
    }

    private boolean isDone() {
        return (high - low) <= precision;
    }

    private Command findHigh(SmartSubsystem subsystem) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    low = high;
                    high *= 2;
                    if (high > max) {
                        throw new IllegalStateException("Could not find upper bound for " + itemBeingCharacterized + " in " + subsystem.getName());
                    }
                }, subsystem),
                Commands.defer(() -> test.apply(high), Set.of(subsystem))
                ).until(() -> !isValid.getAsBoolean());
    }

    private Command checkLow(SmartSubsystem subsystem) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    if (low > max) {
                        throw new IllegalStateException("Low is too large for "  + itemBeingCharacterized + " in " + subsystem.getName());
                    }
                }, subsystem),
                Commands.defer(() -> test.apply(low), Set.of(subsystem)),
                Commands.runOnce(() -> {
                    if (!isValid.getAsBoolean()) {
                        throw new IllegalStateException("Low is too large for " + itemBeingCharacterized + " in " + subsystem.getName());
                    }
                }));
    }

    private Command binarySearch(SmartSubsystem subsystem) {
        return Commands.sequence(
                Commands.defer(() -> test.apply(getMidpoint()), Set.of(subsystem)),
                Commands.runOnce(() -> {
                    if (!isValid.getAsBoolean()) {
                        tooHigh();
                    } else {
                        tooLow();
                    }
                }, subsystem)).until(() -> isDone());
    }

    public Command outputOptimal(SmartSubsystem subsystem) {
        return Commands.runOnce(() -> {
            System.out.println(subsystem.getName() + " " + itemBeingCharacterized + " Optimal Value: " + getMidpoint());
            SmartDashboard.putNumber(subsystem.getName() + "/AutoTune/" + itemBeingCharacterized + "/Current Estimate", getMidpoint());
            Preferences.setDouble(subsystem.getName() + "/AutoTune/" + itemBeingCharacterized + "/Current Estimate", getMidpoint());
            reset();
        }, subsystem);
    }

    public void publishTelemetry(SmartSubsystem subsystem) {
        SmartDashboard.putNumber(subsystem.getName() + "/AutoTune/" + itemBeingCharacterized + "/Low", low);
        SmartDashboard.putNumber(subsystem.getName() + "/AutoTune/" + itemBeingCharacterized + "/High", high);
        SmartDashboard.putNumber(subsystem.getName() + "/AutoTune/" + itemBeingCharacterized + "/Midpoint", getMidpoint());
    }

    public double getEstimateFromSmartDashboard(SmartSubsystem subsystem) {
        return SmartDashboard.getNumber(subsystem.getName() + "/AutoTune/" + itemBeingCharacterized + "/Current Estimate", 0);
    }

    public double getEstimateFromPreferences(SmartSubsystem subsystem) {
        return Preferences.getDouble(subsystem.getName() + "/AutoTune/" + itemBeingCharacterized + "/Current Estimate", 0);
    }

    public double getOptimal() {
        return getMidpoint();
    }

    public Command findOptimal(SmartSubsystem subsystem) {
        return Commands.sequence(
                checkLow(subsystem),
                findHigh(subsystem),
                binarySearch(subsystem),
                outputOptimal(subsystem)
        );
    }
}



//NOTE(to be deleted): the optimal method for a flywheel is to have test as motor.set(midpoint) followed by Commands.waitSeconds(0.5) and isValid as a boolean supplier that checks if the flywheel is at the target speed. 