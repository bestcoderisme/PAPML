package frc.robot.papml;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FFConstants {
    public double kS;
    public double kV;
    public double kA;
    public double kG;

    public FFConstants(double kS, double kV, double kA, double kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    public void publishToPreferences(String prefix) {
        // Publish to preferences
        Preferences.setDouble(prefix + "/kS", kS);
        Preferences.setDouble(prefix + "/kV", kV);
        Preferences.setDouble(prefix + "/kA", kA);
        Preferences.setDouble(prefix + "/kG", kG);
    }

    public void getFromPreferences(String prefix) {
        // Get from preferences
        this.kS = Preferences.getDouble(prefix + "/kS", kS);
        this.kV = Preferences.getDouble(prefix + "/kV", kV);
        this.kA = Preferences.getDouble(prefix + "/kA", kA);
        this.kG = Preferences.getDouble(prefix + "/kG", kG);
    }

    public void publishToSmartDashboard(String prefix){
        SmartDashboard.putNumber(prefix + "/kS", kS);
        SmartDashboard.putNumber(prefix + "/kV", kV);
        SmartDashboard.putNumber(prefix + "/kA", kA);
        SmartDashboard.putNumber(prefix + "/kG", kG);
    }

    public static FFConstants getFFFromPreferences(String prefix) {
        double kS = Preferences.getDouble(prefix + "/kS", 0.0);
        double kV = Preferences.getDouble(prefix + "/kV", 0.0);
        double kA = Preferences.getDouble(prefix + "/kA", 0.0);
        double kG = Preferences.getDouble(prefix + "/kG", 0.0);
        return new FFConstants(kS, kV, kA, kG);
    }
}
