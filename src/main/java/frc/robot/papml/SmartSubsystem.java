package frc.robot.papml;

import java.util.function.DoubleConsumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.papml.abstraction.motor.Motor;

public abstract class SmartSubsystem extends SubsystemBase{
    public enum ControlMode {
        DISABLED,
        NORMAL,
        CALIBRATING_FF,
        CALIBRATING_PID
    }

    private enum TargetSide {
        BELOW,
        ABOVE
    }

    protected TargetSide previousSide;
    protected int oscillations;

    protected Motor motor;
    protected PIDController pid;
    protected String name;
    protected Timer timer = new Timer();
    protected FFRegression regressor;
    protected FFCharacterizationSamples samples;
    protected ControlMode mode;
    protected double target;
    protected SearchAlgorithm searchAlgorithmForkP;
    protected SearchAlgorithm searchAlgorithmForkD;
    protected double accuracyThreshold = 0.005; 
    protected int oscillationLimit;
    protected Debouncer debouncer;


    protected SmartSubsystem(String name, Motor motor, double accuracyThreshold, int oscillationLimit) {
        this.name = name;
        this.motor = motor;
        this.pid = new PIDController(0, 0.0, 0.0);
        this.mode = ControlMode.NORMAL;
        this.accuracyThreshold = accuracyThreshold;
        this.oscillationLimit = oscillationLimit;
        initializeSmartDashboard();
    }

    protected void initializeSearchAlgorithms(){
        searchAlgorithmForkP = createSearchAlgorithm((double constant) -> {
            pid.setP(constant);
        }, "kP");
        searchAlgorithmForkD = createSearchAlgorithm((double constant) -> {
            pid.setD(constant);
        }, "kD");
    }
    public Command regressSamples(){
        return Commands.runOnce(()->{
            regressor = new FFRegression(samples);
            FFConstants coeffs = regressor.getCoefficients();
            coeffs.publishToSmartDashboard(name);
            coeffs.publishToPreferences(name);
            setFFWithPreferences();
        });
    }

    abstract void setFFWithPreferences();

    abstract Command calculateFFGains();

    abstract double getVoltageFromPIDF();

    public void autoTune(){
        calculateFFGains();
    }

    public String getName(){
        return name;
    }

    public void disableSubsystem(){
        mode = ControlMode.DISABLED;
        motor.stop();
    }

    protected void runPID() {
        if (mode == ControlMode.NORMAL || mode == ControlMode.CALIBRATING_PID) {
            motor.setVoltage(getVoltageFromPIDF());
        } else if (mode == ControlMode.DISABLED) {
            motor.stop();
        }
    }    

    protected void publishTelemetry(){
        SmartDashboard.putString(name + "/ControlMode", mode.toString());
    }

    protected void backgroundTasks(){
        runPID();
        publishTelemetry();

        if(mode==ControlMode.CALIBRATING_PID){
            checkOscillation(motor.getVelocity(), target);
            SmartDashboard.putNumber(name + "/AutoTune/Oscillations", oscillations);
        }
    }

    public void periodic() {
        backgroundTasks();
    }

    abstract SearchAlgorithm createSearchAlgorithm(DoubleConsumer setConstant, String constantName);

    protected void checkOscillation(double velocity, double targetRPM) {
        double error = velocity - targetRPM;

        TargetSide side = null;

        if (error > targetRPM * accuracyThreshold) {
            side = TargetSide.ABOVE;
        } else if (error < -targetRPM * accuracyThreshold) {
            side = TargetSide.BELOW;
        }

        if (side != null) {
            if (previousSide != null && side != previousSide) {
                oscillations++;
            }

            previousSide = side;
        }
    }

    protected void initializeSmartDashboard() {
        publishTelemetry();
        SmartDashboard.putNumber(name + "/AutoTune/TargetRPM", 0);
        SmartDashboard.putNumber(name + "/AutoTune/AccuracyThreshold", accuracyThreshold);
        SmartDashboard.putNumber(name + "/AutoTune/Oscillations", oscillations);
    }

    protected boolean isWithinOscillationLimit() {
        return oscillations <= oscillationLimit;
    }

    protected boolean isSettled() {
        return debouncer.calculate(Math.abs(motor.getVelocity() - target) < target * accuracyThreshold);
    }

    protected void resetOscillationTracking() {
        previousSide = null;
        oscillations = 0;
    }
}
