package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IntakeIOTalonFX implements IntakeIO {
  private static final int id = 13; // FIXME
  private static final double currentLimitAmps = 40;
  private static final boolean inverted = false;
  private static final boolean brake = false;
  private static final double reduction = 1 / 1;

  private final TalonFX talon;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> supplyCurrent;

  private final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);
  private final NeutralOut neutralOutput = new NeutralOut();

  private final double mechanismReduction;

  public IntakeIOTalonFX() {
    talon = new TalonFX(id);

    mechanismReduction = reduction;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, appliedVolts, supplyCurrent);

    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent).isOK();
    inputs.positionRads =
        Units.rotationsToRadians(position.getValueAsDouble()) / mechanismReduction;
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(velocity.getValueAsDouble()) / mechanismReduction;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOutput.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }
}
