package frc.robot.papml;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Objects;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.papml.abstraction.encoder.AbsoluteCTREEncoder;
import frc.robot.papml.abstraction.encoder.AbsoluteEncoder;
import frc.robot.papml.abstraction.motor.CTREMotor;
import frc.robot.papml.abstraction.motor.Motor;

public class DrivetrainSubsystem extends SubsystemBase {
    private final EnumMap<ModuleType, Module> modules;
    private final String name;

    public DrivetrainSubsystem(String name) {
        this.name = name;
        this.modules = new EnumMap<>(ModuleType.class);
    }

    private void addModule(Module module) {
        if (modules.containsKey(module.moduleType)) {
            throw new IllegalArgumentException("Module already exists: " + module.moduleType);
        }

        modules.put(module.moduleType, module);
    }

    public DrivetrainSubsystem addFrontLeftModule(Motor driveMotor, Motor steerMotor, AbsoluteEncoder moduleEncoder) {
        addModule(new Module(ModuleType.FRONT_LEFT, driveMotor, steerMotor, moduleEncoder));
        return this;
    }

    public DrivetrainSubsystem addFrontRightModule(Motor driveMotor, Motor steerMotor, AbsoluteEncoder moduleEncoder) {
        addModule(new Module(ModuleType.FRONT_RIGHT, driveMotor, steerMotor, moduleEncoder));
        return this;
    }

    public DrivetrainSubsystem addBackLeftModule(Motor driveMotor, Motor steerMotor, AbsoluteEncoder moduleEncoder) {
        addModule(new Module(ModuleType.BACK_LEFT, driveMotor, steerMotor, moduleEncoder));
        return this;
    }

    public DrivetrainSubsystem addBackRightModule(Motor driveMotor, Motor steerMotor, AbsoluteEncoder moduleEncoder) {
        addModule(new Module(ModuleType.BACK_RIGHT, driveMotor, steerMotor, moduleEncoder));
        return this;
    }

    public void usePAPMLStandardizedCTREModuleConfiguration() {
        addFrontLeftModule(
                new CTREMotor(new TalonFX(8)),
                new CTREMotor(new TalonFX(7)),
                new AbsoluteCTREEncoder(new CANcoder(9)));

        addFrontRightModule(
                new CTREMotor(new TalonFX(11)),
                new CTREMotor(new TalonFX(10)),
                new AbsoluteCTREEncoder(new CANcoder(12)));

        addBackLeftModule(
                new CTREMotor(new TalonFX(2)),
                new CTREMotor(new TalonFX(1)),
                new AbsoluteCTREEncoder(new CANcoder(3)));

        addBackRightModule(
                new CTREMotor(new TalonFX(5)),
                new CTREMotor(new TalonFX(4)),
                new AbsoluteCTREEncoder(new CANcoder(6)));

        validateModules();
    }

    private void validateModules() {
        List<ModuleType> missingModules = new ArrayList<>();

        for (ModuleType type : ModuleType.values()) {
            if (!modules.containsKey(type)) {
                missingModules.add(type);
            }
        }

        if (!missingModules.isEmpty()) {
            throw new IllegalStateException(
                    "Missing drivetrain modules: " + missingModules);
        }
    }

    private static class Module {
        private final Motor driveMotor;
        private final Motor steerMotor;
        private final AbsoluteEncoder moduleEncoder;
        private final ModuleType moduleType;

        public Module(ModuleType moduleType, Motor driveMotor, Motor steerMotor, AbsoluteEncoder moduleEncoder) {
            this.moduleType = Objects.requireNonNull(moduleType);
            this.driveMotor = Objects.requireNonNull(driveMotor);
            this.steerMotor = Objects.requireNonNull(steerMotor);
            this.moduleEncoder = Objects.requireNonNull(moduleEncoder);
        }
    }

    private enum ModuleType {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

}
