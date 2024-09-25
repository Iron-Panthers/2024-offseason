package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;

public class ModuleIOFalcon500 implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX steerTalon;
  private final CANcoder encoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;

  private final Supplier<Rotation2d> steerAbsolutePosition;
  private final StatusSignal<Double> steerPosition;
  private final StatusSignal<Double> steerVelocity;
  private final StatusSignal<Double> steerAppliedVolts;

  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration steerConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  private VelocityVoltage driveVelocityControl = new VelocityVoltage(0).withUpdateFreqHz(0);
  private PositionVoltage steerPositionControl = new PositionVoltage(0).withUpdateFreqHz(0);

  public ModuleIOFalcon500(ModuleConfig config) {
    driveTalon = new TalonFX(config.driveID());
    steerTalon = new TalonFX(config.steerID());
    encoder = new CANcoder(config.encoderID());

    // config
    encoderConfig.MagnetSensor.MagnetOffset = config.absoluteEncoderOffset().getRotations();

    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveConfig.Feedback.SensorToMechanismRatio = MODULE_CONSTANTS.driveReduction();
    steerConfig.Feedback.SensorToMechanismRatio = MODULE_CONSTANTS.steerReduction();
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

    setDriveSlot0(
        MODULE_CONSTANTS.drivekP(),
        0,
        MODULE_CONSTANTS.drivekD(),
        MODULE_CONSTANTS.drivekS(),
        MODULE_CONSTANTS.drivekV(),
        MODULE_CONSTANTS.drivekA());
    setSteerSlot0(
        MODULE_CONSTANTS.steerkP(),
        0,
        MODULE_CONSTANTS.steerkD(),
        MODULE_CONSTANTS.steerkS(),
        MODULE_CONSTANTS.steerkV(),
        MODULE_CONSTANTS.steerkA());

    driveTalon.getConfigurator().apply(driveConfig);
    steerTalon.getConfigurator().apply(steerConfig);
    encoder.getConfigurator().apply(encoderConfig);

    // canbus optimization
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();

    steerAbsolutePosition =
        () ->
            Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble())
                .minus(config.absoluteEncoderOffset());
    steerPosition = steerTalon.getPosition();
    steerVelocity = steerTalon.getVelocity();
    steerAppliedVolts = steerTalon.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        encoder.getAbsolutePosition(),
        steerPosition,
        steerVelocity,
        steerAppliedVolts);

    driveTalon.optimizeBusUtilization();
    steerTalon.optimizeBusUtilization();
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts).isOK();
    inputs.drivePositionRads = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();

    inputs.steerMotorConnected =
        BaseStatusSignal.refreshAll(steerPosition, steerVelocity, steerAppliedVolts).isOK();
    inputs.steerAbsolutePostion = steerAbsolutePosition.get();
    inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValueAsDouble());
    inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
    inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadsPerSec) {
    driveTalon.setControl(
        driveVelocityControl.withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
  }

  @Override
  public void runSteerPositionSetpoint(double angleRads) {
    steerTalon.setControl(steerPositionControl.withPosition(Units.radiansToRotations(angleRads)));
  }

  @Override
  public void setDriveSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    driveConfig.Slot0.kS = kS;
    driveConfig.Slot0.kV = kV;
    driveConfig.Slot0.kA = kA;
    driveTalon.getConfigurator().apply(driveConfig);
  }

  @Override
  public void setSteerSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
    steerConfig.Slot0.kP = kP;
    steerConfig.Slot0.kI = kI;
    steerConfig.Slot0.kD = kD;
    steerConfig.Slot0.kS = kS;
    steerConfig.Slot0.kV = kV;
    steerConfig.Slot0.kA = kA;
    steerTalon.getConfigurator().apply(steerConfig);
  }
}
