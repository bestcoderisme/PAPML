package frc.robot.papml;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;

import frc.robot.papml.FFCharacterizationSamples.FFCharacterizationSample;
import frc.robot.papml.FFCharacterizationSamples.GravityMode;
//add min_velocity and sample size check later
public class FFRegression {

    private OLSMultipleLinearRegression regression;
    private double[] voltages;
    private double [][] inputs;
    FFCharacterizationSamples samples;
    boolean equilibriumTest;
    public FFRegression(FFCharacterizationSamples samples, boolean filterLowVelocity, boolean equilibriumTest) {
        this.samples = samples;
        this.equilibriumTest = equilibriumTest;
        this.regression = new OLSMultipleLinearRegression();
        regression.setNoIntercept(true);
        if(filterLowVelocity) {
            velocityFilter(samples);
        }
        double[] accelerations = calculateAcceleration(samples);
        this.voltages = new double[samples.getSamples().size()-1];
        if (equilibriumTest) {
            this.inputs = new double[samples.getSamples().size() - 1][2];
        } else if (samples.getGravityMode() == GravityMode.NONE) {
            this.inputs = new double[samples.getSamples().size() - 1][3];
        } else {
            this.inputs = new double[samples.getSamples().size() - 1][4];
        }
        //get i+1 to skip first sample which has invalid acceleration reading
        for(int i = 0; i < samples.getSamples().size()-1; i++){
            FFCharacterizationSample sample = samples.getSamples().get(i + 1);

            voltages[i] = sample.voltage;
            if(equilibriumTest){
                inputs[i][0] = 1;
            }
            else{
                inputs[i][0] = Math.signum(sample.velocity);
            }
            if(!equilibriumTest){
                inputs[i][1] = sample.velocity;
                inputs[i][2] = accelerations[i];
            }
            if(samples.getGravityMode() == GravityMode.LINEAR){
                if(equilibriumTest){
                    inputs[i][1] = 1;
                }
                else{
                    inputs[i][3] = 1;
                }
            }
            else if(samples.getGravityMode() == GravityMode.COSINE){
                if(equilibriumTest){
                    inputs[i][1] = Math.cos(sample.position);
                }
                else{
                    inputs[i][3] = Math.cos(sample.position);
                }
            }
        }
        regression.newSampleData(voltages, inputs); 
    }

    public FFConstants getCoefficients() {
        double[] coeffs = regression.estimateRegressionParameters();
        if(samples.getGravityMode() == GravityMode.NONE){
            return new FFConstants(coeffs[0], coeffs[1], coeffs[2], 0);
        }
        if(equilibriumTest){
            return new FFConstants(coeffs[0], 0, 0, coeffs[1]);
        }
        return new FFConstants(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
    }

    private void velocityFilter(FFCharacterizationSamples samples) {
        for(int i=0; i<samples.getSamples().size(); i++){
            if(Math.abs(samples.getSamples().get(i).velocity)<1){
                samples.getSamples().remove(i);
                i--; // Adjust index after removal
            }
        }
    }

    private double[] calculateAcceleration(FFCharacterizationSamples samples) {
        double[] accelerations = new double[samples.getSamples().size()-1];
        for(int i=1; i<samples.getSamples().size(); i++){
            double deltaTime = samples.getSamples().get(i).time - samples.getSamples().get(i-1).time;
            double deltaVelocity = samples.getSamples().get(i).velocity - samples.getSamples().get(i-1).velocity;
            double acceleration = deltaVelocity / deltaTime;
            accelerations[i-1] = acceleration;
        }
        return accelerations;
    }
}
