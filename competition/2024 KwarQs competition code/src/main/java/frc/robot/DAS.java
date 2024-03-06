package frc.robot;

import java.util.NavigableMap;
import java.util.TreeMap;

// DAS means distance angle speed table
public class DAS {
    public class MotorSettings {
        double angle; // in degrees
        double voltage; // in volts

        public MotorSettings(double angle, double voltage) {
            this.angle = angle;
            this.voltage = voltage;
        }

        public double getAngle() {
            return angle;
        }

        public double getVoltage() {
            return voltage;
        }
    }

    private NavigableMap<Double, MotorSettings> distanceMap; // Map from distance to settings

    public DAS() {
        distanceMap = new TreeMap<>();
        initializeMap();
    }

    private void initializeMap() {
        // Example values, replace these with your actual mappings
        distanceMap.put(1.318, new MotorSettings(323, -12)); // fix values -8
        distanceMap.put(1.655, new MotorSettings(321.5, -12)); // fix values -8
        distanceMap.put(2.01, new MotorSettings(317.3, -12));//-8
        distanceMap.put(2.357, new MotorSettings(313, -12)); // -9
        distanceMap.put(2.7, new MotorSettings(310, -12));  // -9
        distanceMap.put(3.01, new MotorSettings(308, -12));  // -12
        distanceMap.put(3.30, new MotorSettings(306, -12)); // -12
    }

    public MotorSettings calculateAS(double distance) {
        // Direct match
        if (distanceMap.containsKey(distance)) {
            return distanceMap.get(distance);
        }

        // Find the closest lower and higher keys for interpolation
        Double lowerKey = distanceMap.lowerKey(distance);
        Double higherKey = distanceMap.higherKey(distance);

        // Edge cases: no lower or higher keys
        if (lowerKey == null) {
            return distanceMap.get(higherKey);
        }
        if (higherKey == null) {
            return distanceMap.get(lowerKey);
        }

        // Interpolation
        MotorSettings lowerSettings = distanceMap.get(lowerKey);
        MotorSettings higherSettings = distanceMap.get(higherKey);

        // Calculate weighted average
        double ratio = (distance - lowerKey) / (higherKey - lowerKey);
        double interpolatedAngle = lowerSettings.getAngle()
                + ratio * (higherSettings.getAngle() - lowerSettings.getAngle());
        double interpolatedVoltage = lowerSettings.getVoltage()
                + ratio * (higherSettings.getVoltage() - lowerSettings.getVoltage());

        return new MotorSettings(interpolatedAngle, interpolatedVoltage);
    }
}
