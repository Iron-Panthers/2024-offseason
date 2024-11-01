package frc.robot.subsystems.rollers.accelerator;

import frc.robot.subsystems.rollers.GenericRollersIOTalonFX;

public class AcceleratorIOTalonFX extends GenericRollersIOTalonFX implements AcceleratorIO {
  private static final int id = 14; // FIXME
  private static final int currentLimitAmps = 40;
  private static final boolean inverted = true;
  private static final boolean brake = false;
  private static final double reduction = 1 / 1;

  public AcceleratorIOTalonFX() {
    super(id, currentLimitAmps, inverted, brake, reduction);
  }
}
