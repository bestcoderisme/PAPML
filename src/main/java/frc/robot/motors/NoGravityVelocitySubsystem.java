package frc.robot.motors;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleFunction;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.motors.FFCharacterizationSamples.GravityMode;

public class NoGravityVelocitySubsystem extends SmartSubsystem {
    private SimpleMotorFeedforward feedforward;
    private double targetRPM;
    private CharacterizationRoutine routine;
    private SearchAlgorithm searchAlgorithmForkP;
    private SearchAlgorithm searchAlgorithmForkD;
    private Debouncer debouncer;
    private double lastTime=Double.POSITIVE_INFINITY;
    private double currentTime;
    
    public NoGravityVelocitySubsystem(String name, Motor motor, CharacterizationConstraints constraints) {
        samples = new FFCharacterizationSamples(GravityMode.NONE);
        this.name = name;
        this.motor = motor;
        this.pid = new PIDController(0, 0.0, 0.0);
        FFConstants FFconstants =  FFConstants.getFFFromPreferences(name);
        this.feedforward = new SimpleMotorFeedforward(FFconstants.kS, FFconstants.kV, FFconstants.kA);
        this.routine = new CharacterizationRoutine(samples, constraints, motor);

        this.debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

        searchAlgorithmForkP = createSearchAlgorithm((double constant) -> {
            pid.setP(constant);
        }, "kP");
        searchAlgorithmForkD = createSearchAlgorithm((double constant) -> {
            pid.setD(constant);
        }, "kD");

        // SmartDashboard.putNumber(name+" subsystem kS", feedforward.getKs());
        // SmartDashboard.putNumber(name+" subsystem kV", feedforward.getKv());
        // SmartDashboard.putNumber(name+" subsystem kA", feedforward.getKa());
        // SmartDashboard.putNumber(name+" subsystem kP", pid.getP());
        // SmartDashboard.putNumber(name+" subsystem kI", pid.getI());
        // SmartDashboard.putNumber(name+" subsystem kD", pid.getD());
        // SmartDashboard.putNumber(name+" subsystem targetRPM", targetRPM);
    }

    private SearchAlgorithm createSearchAlgorithm(DoubleConsumer setConstant, String constantName){
        DoubleFunction<Command> test =(double constant) -> 

        Commands.sequence(
            Commands.waitUntil(() -> Math.abs(motor.getVelocity()) < 10),
            Commands.runOnce(
                () -> {
                setConstant.accept(constant);
                timer.restart();
                debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
                setTargetRPM(targetRPM);
                }
            ),
            Commands.waitUntil(
                () -> {
                    return debouncer.calculate(Math.abs(motor.getVelocity() - targetRPM) < targetRPM * 0.01);
                }
            ).withTimeout(3),
            Commands.runOnce(
                () -> {
                    lastTime = currentTime;
                    currentTime = timer.get();
                    setTargetRPM(0);
                    motor.stop();
                }
            )
        );
        BooleanSupplier isValid = () ->  {return currentTime < lastTime;};


        return new SearchAlgorithm(
            0.0005,
            0.001,
            test,
            isValid,
            10.0,
            constantName
        );
    }

    public Command calculatePIDGains(){
        return Commands.sequence(
            searchAlgorithmForkP.findOptimal(this)
        );
    }

    public Command calculateFFGains(){
        return routine.quasistaticRoutine(this, false);
        // return Commands.sequence(
        //     routine.runFullRoutine(this),
        //     regressSamples()
        // );
    }

    protected double getVoltageFromPIDF(){
        return pid.calculate(motor.getEncoder().getVelocity(), targetRPM) + feedforward.calculate(targetRPM);
    }

    @Override
    void setFFWithPreferences() {
        FFConstants FFconstants = FFConstants.getFFFromPreferences(name);
        feedforward = new SimpleMotorFeedforward(FFconstants.kS, FFconstants.kV, FFconstants.kA);
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        SmartDashboard.putNumber(name + "/TargetRPM", targetRPM);
        motor.setVoltage(getVoltageFromPIDF());
    }

    public void publishTelemetry(){
        routine.publishTelemetry(name);
        SmartDashboard.putNumber(name + "/CurrentRPM", motor.getVelocity());
        SmartDashboard.putNumber(name + "/AutoTune/Samples", samples.getSamples().size());
    }

    @Override
    public void periodic() {
        publishTelemetry();
    }
}
