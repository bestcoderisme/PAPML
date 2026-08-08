package frc.robot.papml;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.papml.abstraction.motor.Motor;

public class CharacterizationRoutine {
    private FFCharacterizationSamples samples;
    private CharacterizationConstraints constraints;
    private Timer timer;
    private Motor motor;
    private double curVoltage;
    private final double settledConstant = 0.1; //acceleration threshold for dynamic routine
    private Debouncer debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    public CharacterizationRoutine(FFCharacterizationSamples samples, CharacterizationConstraints constraints, Motor motor) {
        this.samples = samples;
        this.constraints = constraints;
        this.timer = new Timer();
        this.motor = motor;
    }

    public Command quasistaticRoutine(SmartSubsystem subsystem, boolean inReverse) {
        
        return Commands.sequence(
            Commands.runOnce(()->{
                timer.restart();
                curVoltage=0;
            }),

            Commands.run(() -> {
                double time = timer.get();

                if(!inReverse){
                    curVoltage = time * constraints.quasistaticRampRate;
                }
                else{
                    curVoltage = -time * constraints.quasistaticRampRate;
                }

                motor.setVoltage(curVoltage);

                samples.addSample(
                    time,
                    motor.getPosition(),
                    motor.getVelocity(),
                    curVoltage
                );

            }, subsystem)
            .until(() ->
                timer.hasElapsed(constraints.timeLimit)
                || Math.abs(motor.getVelocity()) >= constraints.maxVelocity
                || Math.abs(curVoltage) >= constraints.maxVoltage
            ),

            Commands.runOnce(() -> {
                motor.setVoltage(0);
                timer.stop();
            })
        );
    }

public Command dynamicRoutine(SmartSubsystem subsystem, boolean inReverse) {

    return Commands.sequence(
        Commands.runOnce(() -> {
            timer.restart();
            debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
        }),

        Commands.run(() -> {

            motor.setVoltage(inReverse ? -constraints.dynamicVoltage : constraints.dynamicVoltage);

            double time = timer.get();

            samples.addSample(
                time,
                motor.getPosition(),
                motor.getVelocity(),
                inReverse ? -constraints.dynamicVoltage : constraints.dynamicVoltage
            );

        }, subsystem)
        .until(() ->
            timer.hasElapsed(constraints.timeLimit)
            || Math.abs(motor.getVelocity()) >= constraints.maxVelocity
            || settled() //add another stop condition acceleration
        ),

        Commands.runOnce(() -> {
            motor.setVoltage(0);
            timer.stop();
        })
    );
}

    public Command runFullRoutine(SmartSubsystem subsystem) {
        return Commands.sequence(
            quasistaticRoutine(subsystem, false),
            dynamicRoutine(subsystem, false),
            quasistaticRoutine(subsystem, true),
            dynamicRoutine(subsystem, true)
        );
    }


    public void publishTelemetry(String name){
        SmartDashboard.putNumber(name + "/AutoTune/Voltage", curVoltage);
    }

    private double getAcceleration() {
        if (samples.getSamples().size() < 2) {
            return Double.POSITIVE_INFINITY;
        }

        double velocityBefore = samples.getSamples().get(samples.getSamples().size() - 2).velocity;
        double velocityAfter = samples.getSamples().get(samples.getSamples().size() - 1).velocity;
        double timeInterval = samples.getSamples().get(samples.getSamples().size() - 1).time
                            - samples.getSamples().get(samples.getSamples().size() - 2).time;

        return (velocityAfter - velocityBefore) / timeInterval;
    }    


    private boolean settled(){
        return debouncer.calculate(Math.abs(getAcceleration()) < settledConstant);
    }
}
