package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.DrivetrainConstants.angleEnableCurrentLimit, 
            Constants.DrivetrainConstants.angleContinuousCurrentLimit, 
            Constants.DrivetrainConstants.anglePeakCurrentLimit, 
            Constants.DrivetrainConstants.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.DrivetrainConstants.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.DrivetrainConstants.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.DrivetrainConstants.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.DrivetrainConstants.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.DrivetrainConstants.driveEnableCurrentLimit, 
            Constants.DrivetrainConstants.driveContinuousCurrentLimit, 
            Constants.DrivetrainConstants.drivePeakCurrentLimit, 
            Constants.DrivetrainConstants.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.DrivetrainConstants.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.DrivetrainConstants.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.DrivetrainConstants.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.DrivetrainConstants.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.DrivetrainConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.DrivetrainConstants.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.DrivetrainConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}