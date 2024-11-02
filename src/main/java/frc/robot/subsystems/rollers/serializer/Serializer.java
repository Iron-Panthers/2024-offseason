package frc.robot.subsystems.rollers.serializer;

import frc.robot.subsystems.rollers.GenericRollers;

public class Serializer extends GenericRollers<Serializer.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(6),
    SHOOT_SPEAKER(0),
    SHOOT_AMP(3),
    SPEAKER_TRANSFER(5),
    AMP_TRANSFER(-5),
    EJECT(0);

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
