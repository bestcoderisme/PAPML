package frc.robot.motors;

    public class CharacterizationConstraints {
        public final double maxVoltage;
        public final double maxVelocity;
        public final double timeLimit; // Optional time limit for the characterization routine
        public final double quasistaticRampRate;
        public final double dynamicVoltage;
        public final RangeConstraints rangeConstraints;
        

        public static CharacterizationConstraints createDefaultFlywheel() {
            return new CharacterizationConstraints(12, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.5, 12, null);
        }

        private CharacterizationConstraints(double maxVoltage, double maxVelocity, double timeLimit, double quasistaticRampRate, double dynamicVoltage, RangeConstraints rangeConstraints) {
            if(maxVoltage <= 0) {
                throw new IllegalArgumentException("maxVoltage must be greater than 0");
            }
            if(maxVelocity <= 0) {
                throw new IllegalArgumentException("maxVelocity must be greater than 0");
            }
            if(quasistaticRampRate <= 0) {
                throw new IllegalArgumentException("quasistaticRampRate must be greater than 0");
            }
            if(dynamicVoltage <= 0) {
                throw new IllegalArgumentException("dynamicVoltage must be greater than 0");
            }
            if(timeLimit <= 0) {
                throw new IllegalArgumentException("timeLimit must be greater than 0");
            }
            if(dynamicVoltage > maxVoltage) {
                throw new IllegalArgumentException("dynamicVoltage must be less than or equal to maxVoltage");
            }
            if(dynamicVoltage>12){
                throw new IllegalArgumentException("dynamicVoltage must be less than 12V for battery safety");
            }
            if(maxVoltage>12){
                throw new IllegalArgumentException("maxVoltage must be less than 12V for battery safety");
            }
            this.maxVoltage = maxVoltage;
            this.maxVelocity = maxVelocity;
            this.quasistaticRampRate = quasistaticRampRate;
            this.dynamicVoltage = dynamicVoltage;
            this.timeLimit = timeLimit;
            this.rangeConstraints = rangeConstraints;
        }

        public CharacterizationConstraints withMaxVoltage(double maxVoltage) {
            return new CharacterizationConstraints(
                    maxVoltage,
                    this.maxVelocity,
                    this.timeLimit,
                    this.quasistaticRampRate,
                    this.dynamicVoltage,
                    this.rangeConstraints);
        }

        public CharacterizationConstraints withMaxVelocity(double maxVelocity) {
            return new CharacterizationConstraints(
                    this.maxVoltage,
                    maxVelocity,
                    this.timeLimit,
                    this.quasistaticRampRate,
                    this.dynamicVoltage,
                    this.rangeConstraints);
        }

        public CharacterizationConstraints withQuasistaticRampRate(double quasistaticRampRate) {
            return new CharacterizationConstraints(
                    this.maxVoltage,
                    this.maxVelocity,
                    this.timeLimit,
                    quasistaticRampRate,
                    this.dynamicVoltage,
                    this.rangeConstraints);
        }

        public CharacterizationConstraints withDynamicVoltage(double dynamicVoltage) {
            return new CharacterizationConstraints(
                    this.maxVoltage,
                    this.maxVelocity,
                    this.timeLimit,
                    this.quasistaticRampRate,
                    dynamicVoltage,
                    this.rangeConstraints);
        }

        public CharacterizationConstraints withRange(double minPosition, double maxPosition) {
            return new CharacterizationConstraints(
                    this.maxVoltage,
                    this.maxVelocity,
                    this.timeLimit,
                    this.quasistaticRampRate,
                    this.dynamicVoltage,
                    new RangeConstraints(minPosition, maxPosition));
        }

        public CharacterizationConstraints withTimeLimit(double seconds) {
            return new CharacterizationConstraints(
                this.maxVoltage,
                this.maxVelocity,
                seconds,
                this.quasistaticRampRate,
                this.dynamicVoltage,
                this.rangeConstraints);
        }

        public boolean hasRangeConstraints() {
            return rangeConstraints != null;
        }

        public static class RangeConstraints {
            public final double minPosition;
            public final double maxPosition;

            public RangeConstraints(double minPosition, double maxPosition) {
                if (minPosition >= maxPosition) {
                    throw new IllegalArgumentException("minPosition must be less than maxPosition");
                }
                this.minPosition = minPosition;
                this.maxPosition = maxPosition;
            }  
        }
    }
