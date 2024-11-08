package frc.robot.subsystems.rollers.serializer;

import frc.robot.subsystems.rollers.GenericRollers;

public class Serializer extends GenericRollers<Serializer.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(3),
    SHOOT_SPEAKER(0),
    SHOOT_AMP(-8),
    SPEAKER_TRANSFER(3),
    AMP_TRANSFER(-2),
    EJECT(-1);

    private int volts;

    private Target(int volts) {
      this.volts = volts;
    }

    public int getVolts() {
      return volts;
    }
  }

  public Serializer(SerializerIO serializerIO) {
    super("Serializer", serializerIO);
    setVoltageTarget(Target.IDLE);
  }
}
