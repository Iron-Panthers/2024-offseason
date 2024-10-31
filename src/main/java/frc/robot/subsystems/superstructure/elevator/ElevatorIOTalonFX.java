package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX talon;

  private final StatusSignal<Double> positionRotations;
  private final StatusSignal<Double> velocityRPS;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> supplyCurrent;
  private final StatusSignal<Double> temp;

  private final PositionVoltage positionControl = new PositionVoltage(0).withUpdateFreqHz(0);
  private final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);
  private final NeutralOut neutralOutput = new NeutralOut();
  
  public ElevatorIOTalonFX() {
    talon = new TalonFX(ElevatorConstants.ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        ElevatorConstants.INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    positionRotations = talon.getPosition();
    velocityRPS = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50, positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp).isOK();
    inputs.positionRotations = positionRotations.getValueAsDouble();
    inputs.velocityRotPerSec = velocityRPS.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void runPosition(double rotations) {
    talon.setControl(positionControl.withPosition(rotations));
  }

  @Override
  public void runCharacterization(double volts) {
    talon.setControl(voltageOutput.withOutput(volts));
  }
  
  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }
}

