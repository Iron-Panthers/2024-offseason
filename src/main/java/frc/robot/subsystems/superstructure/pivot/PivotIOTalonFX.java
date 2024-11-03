package frc.robot.subsystems.superstructure.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class PivotIOTalonFX implements PivotIO {
  private final TalonFX talon;
  private final CANcoder canCoder;

  private final PIDController pivotPID;

  private final StatusSignal<Double> positionRotations;
  private final StatusSignal<Double> velocityRPS;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> supplyCurrent;
  private final StatusSignal<Double> temp;

  private final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);
  private final NeutralOut neutralOutput = new NeutralOut();

  public PivotIOTalonFX() {
    talon = new TalonFX(PivotConstants.ID);
    canCoder = new CANcoder(PivotConstants.ENCODER_ID);
    canCoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                        .withMagnetOffset(0)));
    canCoder.getConfigurator().setPosition(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        PivotConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = PivotConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);
    talon.setPosition(0);
    talon.setNeutralMode(NeutralModeValue.Brake);

    positionRotations = canCoder.getPosition();
    velocityRPS = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();

    pivotPID = new PIDController(PivotConstants.P, PivotConstants.I, PivotConstants.D);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp)
            .isOK();
    inputs.positionRotations = positionRotations.getValueAsDouble() * 360;
    inputs.velocityRotPerSec = velocityRPS.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void runPosition(double position) {
    double pidOutput =
        pivotPID.calculate(getPosition(), MathUtil.clamp(position, 0, PivotConstants.UPPER_LIMIT));
    talon.setControl(
        voltageOutput.withOutput(
            MathUtil.clamp(
                pidOutput
                    + ((Math.abs(pidOutput) > 0.04) ? PivotConstants.S * Math.signum(pidOutput) : 0)
                    + getFeedforward(),
                PivotConstants.LOWER_VOLT_LIMIT,
                PivotConstants.UPPER_VOLT_LIMIT)));
  }

  @Override
  public void runCharacterization(double volts) {
    talon.setControl(voltageOutput.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }

  private double getPosition() {
    return canCoder.getPosition().getValueAsDouble() * 360 / PivotConstants.REDUCTION;
  }

  private double getFeedforward() {
    return Math.cos(Math.toRadians(canCoder.getPosition().getValueAsDouble() * 360))
        * PivotConstants.G;
  }

  public void setOffset() {
    talon.setPosition(0);
  }
}
