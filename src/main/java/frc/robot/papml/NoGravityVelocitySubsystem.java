package frc.robot.papml;


import java.util.concurrent.atomic.AtomicBoolean;
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
import frc.robot.papml.FFCharacterizationSamples.GravityMode;
import frc.robot.papml.abstraction.motor.Motor;

public class NoGravityVelocitySubsystem extends SmartSubsystem {

    private SimpleMotorFeedforward feedforward;
    private CharacterizationRoutine routine;
    private Debouncer debouncer;
    private double lastTime=Double.POSITIVE_INFINITY;
    private double currentTime=Double.POSITIVE_INFINITY;

    
    public NoGravityVelocitySubsystem(String name, Motor motor, CharacterizationConstraints constraints) {
        super(name, motor);
        samples = new FFCharacterizationSamples(GravityMode.NONE);
        FFConstants FFconstants =  FFConstants.getFFFromPreferences(name);
        this.feedforward = new SimpleMotorFeedforward(FFconstants.kS, FFconstants.kV, FFconstants.kA);
        this.routine = new CharacterizationRoutine(samples, constraints, motor);

        this.debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

        initializeSearchAlgorithms();


        // SmartDashboard.putNumber(name+" subsystem kS", feedforward.getKs());
        // SmartDashboard.putNumber(name+" subsystem kV", feedforward.getKv());
        // SmartDashboard.putNumber(name+" subsystem kA", feedforward.getKa());
        // SmartDashboard.putNumber(name+" subsystem kP", pid.getP());
        // SmartDashboard.putNumber(name+" subsystem kI", pid.getI());
        // SmartDashboard.putNumber(name+" subsystem kD", pid.getD());
        // SmartDashboard.putNumber(name+" subsystem targetRPM", targetRPM);
    }

    @Override
    protected SearchAlgorithm createSearchAlgorithm(DoubleConsumer setConstant, String constantName){
        AtomicBoolean reachedCalibrationTarget = new AtomicBoolean(false);
        DoubleFunction<Command> test =(double constant) -> 
        Commands.sequence(
            Commands.runOnce(()->
            {
                setTargetRPM(0);
                reachedCalibrationTarget.set(false);
            }
            ),
            Commands.waitUntil(() -> Math.abs(motor.getVelocity()) < 10),
            Commands.runOnce(
                () -> {
                setConstant.accept(constant);
                setTargetRPM(3000);
                timer.restart();
                debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
                }
            ),
            Commands.waitUntil(
                () -> {
                    reachedCalibrationTarget.set(debouncer.calculate(Math.abs(motor.getVelocity() - target) < target * 0.005));
                    return reachedCalibrationTarget.get();
                }
            ).withTimeout(3),
            Commands.runOnce(
                () -> {
                    lastTime = currentTime;
                    currentTime = reachedCalibrationTarget.get() ? timer.get() : 3;
                    setTargetRPM(0);
                    motor.stop();
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

    public Command calculatePIDGains(){
        if(mode==ControlMode.DISABLED)
            return Commands.none();
        return Commands.sequence(
            Commands.runOnce(()-> mode=ControlMode.CALIBRATING_PID, this),
            // setTargetRPMCmd(targetRPMForCalibration),
            searchAlgorithmForkP.findOptimal(this)
        ).finallyDo(()-> mode=ControlMode.NORMAL);
    }

    public Command calculateFFGains(){
        if(mode==ControlMode.DISABLED)
            return Commands.none();
        return Commands.runOnce(()-> mode=ControlMode.CALIBRATING_FF, this)
        .andThen(routine.quasistaticRoutine(this, false))
        .andThen(routine.quasistaticRoutine(this, true))
        .andThen(routine.dynamicRoutine(this, false))
        .andThen(routine.dynamicRoutine(this, true))
        .finallyDo(()-> mode=ControlMode.NORMAL);
        // return Commands.sequence(
        //     routine.runFullRoutine(this),
        //     regressSamples()
        // );
    }

    protected double getVoltageFromPIDF(){
        return pid.calculate(motor.getEncoder().getVelocity(), target) + feedforward.calculate(target);
    }

    @Override
    void setFFWithPreferences() {
        FFConstants FFconstants = FFConstants.getFFFromPreferences(name);
        feedforward = new SimpleMotorFeedforward(FFconstants.kS, FFconstants.kV, FFconstants.kA);
    }

    public void setTargetRPM(double target) {
        this.target = target;
        SmartDashboard.putNumber(name + "/TargetRPM", target);
    }

    public Command setTargetRPMCmd(double target) {
        return Commands.runOnce(() -> {
            setTargetRPM(target);
        }, this);
    }

    @Override
    public void publishTelemetry(){
        super.publishTelemetry();
        routine.publishTelemetry(name);
        searchAlgorithmForkP.publishTelemetry(this);
        SmartDashboard.putNumber(name + "/CurrentRPM", motor.getVelocity());
        SmartDashboard.putNumber(name + "/AutoTune/Samples", samples.getSamples().size());
    }
}
