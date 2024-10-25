package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class FlywheelsIOTalonFX implements FlywheelsIO {
  private final TalonFX topTalon;
  private final TalonFX bottomTalon;

  private final StatusSignal<Double> topPositionRads;
  private final StatusSignal<Double> topVelocityRPM;
  private final StatusSignal<Double> topAppliedVolts;
  private final StatusSignal<Double> topSupplyCurrent;
  private final StatusSignal<Double> topTemp;
  private final StatusSignal<Double> bottomPositionRads;
  private final StatusSignal<Double> bottomVelocityRPM;
  private final StatusSignal<Double> bottomAppliedVolts;
  private final StatusSignal<Double> bottomSupplyCurrent;
  private final StatusSignal<Double> bottomTemp;

  private final Slot0Configs gainsConfig = new Slot0Configs();
  private final VelocityVoltage velocityOutput = new VelocityVoltage(0).withUpdateFreqHz(0);
  private final NeutralOut neutralOutput = new NeutralOut();

  public FlywheelsIOTalonFX() {
    topTalon = new TalonFX(FLYWHEEL_CONFIG.topID());
    bottomTalon = new TalonFX(FLYWHEEL_CONFIG.bottomID());

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0; // FIXME
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = FLYWHEEL_CONFIG.reduction();

    gainsConfig.kP = GAINS.kP();
    gainsConfig.kI = GAINS.kI();
    gainsConfig.kD = GAINS.kD();
    gainsConfig.kS = GAINS.kS();
    gainsConfig.kV = GAINS.kV();
    gainsConfig.kA = GAINS.kA();

    topTalon.getConfigurator().apply(config);
    bottomTalon.getConfigurator().apply(config);
    topTalon.getConfigurator().apply(gainsConfig);
    bottomTalon.getConfigurator().apply(gainsConfig);

    topTalon.setInverted(false);
    topTalon.setInverted(false); // FIXME

    topPositionRads = topTalon.getPosition();
    topVelocityRPM = topTalon.getVelocity();
    topAppliedVolts = topTalon.getMotorVoltage();
    topSupplyCurrent = topTalon.getSupplyCurrent();
    topTemp = topTalon.getDeviceTemp();

    bottomPositionRads = bottomTalon.getPosition();
    bottomVelocityRPM = bottomTalon.getVelocity();
    bottomAppliedVolts = bottomTalon.getMotorVoltage();
    bottomSupplyCurrent = bottomTalon.getSupplyCurrent();
    bottomTemp = bottomTalon.getDeviceTemp();
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.topMotorConnected =
        BaseStatusSignal.refreshAll(
                topPositionRads, topVelocityRPM, topAppliedVolts, topSupplyCurrent, topTemp)
            .isOK();
    inputs.bottomMotorConnected =
        BaseStatusSignal.refreshAll(
                bottomPositionRads,
                bottomVelocityRPM,
                bottomAppliedVolts,
                bottomSupplyCurrent,
                bottomTemp)
            .isOK();

    inputs.topPositionRads = Units.rotationsToRadians(topPositionRads.getValueAsDouble());
    inputs.topVelocityRPM = topVelocityRPM.getValueAsDouble() * 60.0;
    inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
    inputs.topSupplyCurrent = topSupplyCurrent.getValueAsDouble();
    inputs.topTempCelcius = topTemp.getValueAsDouble();

    inputs.bottomPositionRads = Units.rotationsToRadians(bottomPositionRads.getValueAsDouble());
    inputs.bottomVelocityRPM = bottomVelocityRPM.getValueAsDouble() * 60.0;
    inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.bottomSupplyCurrent = bottomSupplyCurrent.getValueAsDouble();
    inputs.bottomTempCelcius = bottomTemp.getValueAsDouble();
  }

  @Override
  public void runVelocity(int topVelocity, int bottomVelocity) {
    topTalon.setControl(velocityOutput.withVelocity(topVelocity / 60));
    bottomTalon.setControl(velocityOutput.withVelocity(bottomVelocity / 60));
  }

  @Override
  public void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
    gainsConfig.kP = kP;
    gainsConfig.kI = kI;
    gainsConfig.kD = kD;
    gainsConfig.kS = kS;
    gainsConfig.kV = kV;
    gainsConfig.kA = kA;

    topTalon.getConfigurator().apply(gainsConfig);
    bottomTalon.getConfigurator().apply(gainsConfig);
  }

  @Override
  public void stop() {
    topTalon.setControl(neutralOutput);
    bottomTalon.setControl(neutralOutput);
  }
}
