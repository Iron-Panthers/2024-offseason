package frc.robot.subsystems.rollers.serializer;

import frc.robot.subsystems.rollers.GenericRollers;

public class Serializer extends GenericRollers<Serializer.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(12),
    EJECT(-8);

    private int volts;

    private Target(int volts) {
      this.volts = volts;
    }

    public int getVolts() {
      return volts;
    }
  }

  private Target voltageTarget = Target.IDLE;

  public Serializer(SerializerIO serializerIO) {
    super("Serializer", serializerIO);
  }
}
