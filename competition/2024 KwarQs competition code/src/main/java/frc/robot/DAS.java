package frc.robot;

import java.util.NavigableMap;
import java.util.TreeMap;

// DAS means distance angle speed table
public class DAS {
    public class MotorSettings {
        double angle; // in degrees
        double velocity; // in volts

        public MotorSettings(double angle, double velocity) {
            this.angle = angle;
            this.velocity = velocity;
        }

        public double getAngle() {
            return angle;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    private NavigableMap<Double, MotorSettings> distanceMap; // Map from distance to settings

    public DAS() {
        distanceMap = new TreeMap<>();
        initializeMap();
    }

    private void initializeMap() {
        // Example values, replace these with your actual mappings
        distanceMap.put(1.318, new MotorSettings(323, -5300)); // fix values -8
        distanceMap.put(1.655, new MotorSettings(321.5, -5300)); // fix values -8
        distanceMap.put(2.01, new MotorSettings(317.3, -5300));//-8
        distanceMap.put(2.357, new MotorSettings(313, -5300)); // -9
        distanceMap.put(2.7, new MotorSettings(310, -5300));  // -9
        distanceMap.put(3.01, new MotorSettings(306, -5300));  // -12
        distanceMap.put(3.30, new MotorSettings(305, -5300)); // -12
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
        double interpolatedVoltage = lowerSettings.getVelocity()
                + ratio * (higherSettings.getVelocity() - lowerSettings.getVelocity());

        return new MotorSettings(interpolatedAngle, interpolatedVoltage);
    }
}
