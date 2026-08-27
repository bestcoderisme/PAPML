package frc.robot.papml;

import java.util.function.DoubleConsumer;

import edu.wpi.first.math.controller.PIDController;
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


    protected SmartSubsystem(String name, Motor motor) {
        this.name = name;
        this.motor = motor;
        this.pid = new PIDController(0, 0.0, 0.0);
        this.mode = ControlMode.NORMAL;
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
    }

    public void periodic() {
        backgroundTasks();
    }

    abstract SearchAlgorithm createSearchAlgorithm(DoubleConsumer setConstant, String constantName);

}
