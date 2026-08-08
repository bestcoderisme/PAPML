package frc.robot.motors;

import java.util.ArrayList;
import java.util.List;

public class FFCharacterizationSamples {

    public enum GravityMode {
        NONE,
        COSINE,
        LINEAR
    }

    static class FFCharacterizationSample{
        public final double time;
        public final double position;
        public final double velocity;
        public final double voltage;

        private FFCharacterizationSample(double time, double position, double velocity, double voltage){
            this.time = time;
            this.position = position;
            this.velocity = velocity;
            this.voltage = voltage;
        }
    }

    private List<FFCharacterizationSample> samples;
    private GravityMode gravityMode;

    public FFCharacterizationSamples(GravityMode gravityMode){
        this.gravityMode = gravityMode;
        this.samples = new ArrayList<>();
    }

    public void addSample(double time, double position, double velocity, double voltage){
        samples.add(new FFCharacterizationSample(time, position, velocity, voltage));
    }

    public List<FFCharacterizationSample> getSamples(){
        return samples;
    }

    public GravityMode getGravityMode() {
        return gravityMode;
    }
}
