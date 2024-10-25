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
        //distanceMap.put(/*distance*/1.23, new MotorSettings(/*angle*/321, -5200));
        distanceMap.put(/*distance*/1.32, new MotorSettings(/*angle*/321.5, -5200));
        distanceMap.put(/*distance*/1.6, new MotorSettings(/*angle*/317, -5200)); //~11 in
        distanceMap.put(/*distance*/1.71, new MotorSettings(/*angle*/315, -5200));//~15.5 in
        distanceMap.put(/*distance*/1.9, new MotorSettings(/*angle*/311, -5200));
        distanceMap.put(/*distance*/2.0, new MotorSettings(/*angle*/309, -5200));//28
        distanceMap.put(/*distance*/2.21, new MotorSettings(/*angle*/307, -5200));//32
        distanceMap.put(/*distance*/2.4, new MotorSettings(/*angle*/303.5, -5200)); //40
        // distanceMap.put(/*distance*/2.7, new MotorSettings(/*angle*/397.5, -5200)); //50
        distanceMap.put(/*distance*/3.01, new MotorSettings(/*angle*/300, -5200));//65in
        distanceMap.put(/*distance*/3.15, new MotorSettings(/*angle*/297, -5200));//70in
        distanceMap.put(/*distance*/3.502, new MotorSettings(/*angle*/293, -5200));//84



        // distanceMap.put(1.318, new MotorSettings(323, -5200)); // fix values -8
        // distanceMap.put(1.655, new MotorSettings(321.5, -5200)); // fix values -8
        // distanceMap.put(2.01, new MotorSettings(317.3, -5200));// -8
        // distanceMap.put(2.357, new MotorSettings(313, -5200)); // -9
        // distanceMap.put(2.7, new MotorSettings(310, -5200)); // -9
        // distanceMap.put(3.01, new MotorSettings(306, -5200)); // -12
        // distanceMap.put(3.20, new MotorSettings(304.5, -5200)); // -12
        // distanceMap.put(3.40, new MotorSettings(304, -5200)); // -12
        // distanceMap.put(3.50, new MotorSettings(303, -5200)); // -12
        // distanceMap.put(3.60, new MotorSettings(301, -5200));
        // distanceMap.put(3.7, new MotorSettings(300.75, -5200));
        // distanceMap.put(4.5, new MotorSettings(297, -5200)); // -12 300.75
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
