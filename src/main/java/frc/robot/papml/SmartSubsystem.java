package frc.robot.papml;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.papml.abstraction.motor.Motor;

public abstract class SmartSubsystem extends SubsystemBase{
    protected Motor motor;
    protected PIDController pid;
    protected String name;
    protected Timer timer = new Timer();
    protected FFRegression regressor;
    protected FFCharacterizationSamples samples;

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


}
