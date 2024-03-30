package frc.robot;

public class Bleh {
    private double[] a = new double[3];

    public Bleh(double[] a) {
        this.a = a;
    }

    public double[] getA() {
        return a;
    }

    /** Translation3d struct for serialization. */
    public static final BlehStruct struct = new BlehStruct();
}
