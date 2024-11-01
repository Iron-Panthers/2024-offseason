package frc.robot.subsystems.rollers.serializer;

import frc.robot.subsystems.rollers.GenericRollersIOTalonFX;

public class SerializerIOTalonFX extends GenericRollersIOTalonFX implements SerializerIO {
  private static final int id = 20; // FIXME
  private static final int currentLimitAmps = 40;
  private static final boolean inverted = true;
  private static final boolean brake = false;
  private static final double reduction = 1 / 1;

  public SerializerIOTalonFX() {
    super(id, currentLimitAmps, inverted, brake, reduction);
  }
}
