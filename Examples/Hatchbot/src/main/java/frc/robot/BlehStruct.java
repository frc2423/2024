package frc.robot;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class BlehStruct implements Struct<Bleh> {
  @Override
  public Class<Bleh> getTypeClass() {
    return Bleh.class;
  }

  @Override
  public String getTypeString() {
    return "struct:Bleh";
  }

  @Override
  public int getSize() {
    return kSizeDouble * 3;
  }

  @Override
  public String getSchema() {
    return "double a[3]";
  }

  @Override
  public Bleh unpack(ByteBuffer bb) {
    double a1 = bb.getDouble();
    double a2 = bb.getDouble();
    double a3 = bb.getDouble();
    return new Bleh(new double[] {a1, a2, a3});
  }

  @Override
  public void pack(ByteBuffer bb, Bleh value) {
    double[] a = value.getA();
    bb.putDouble(a[0]);
    bb.putDouble(a[1]);
    bb.putDouble(a[2]);
  }
}
