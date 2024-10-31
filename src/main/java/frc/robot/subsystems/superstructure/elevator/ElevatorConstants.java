package frc.robot.subsystems.superstructure.elevator;

public class ElevatorConstants {
  public static final double REDUCTION = 1 / 1; // FIXME
  public static final boolean INVERTED = false; // FIXME

  public static final double SUPPLY_CURRENT_LIMIT = 30; // FIXME
  public static final int ZEROING_CURRENT_LIMIT = 20; // FIXME

  public static final int ZEROING_VOLTS = -4; // FIXME

  public static final int ID = -1; // FIXME

  public record PIDGains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}

