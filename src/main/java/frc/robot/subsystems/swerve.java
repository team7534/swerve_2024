package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
  
public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;

    private PositionDutyCycle turnControl = new PositionDutyCycle(0, 1, false, 0, 0, false, false, false);
    private VelocityVoltage velocityControl = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    public TalonFX mAngleMotor;
    public TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    public CTREConfigs ctreConfigs = new CTREConfigs();
      
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
  
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, Constants.Canivore1);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.Canivore1);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.Canivore1);
        configDriveMotor();
    }



    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.setControl(velocityControl.withVelocity(velocity).withFeedForward(0.00008));
        }
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(turnControl.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }
  


    private void resetToAbsolute(){
        double absolutePosition = getCanCoder().getRotations() - angleOffset;
        mAngleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){    
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(ctreConfigs.swerveCanCoderConfig);  
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(new TalonFXConfiguration());
        mAngleMotor.getConfigurator().apply(ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        resetToAbsolute();
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
    }

    private void configDriveMotor(){
        mDriveMotor.getConfigurator().apply(new TalonFXConfiguration());     
        mDriveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(NeutralModeValue.Brake);
        mDriveMotor.setPosition(0);
    }

    public void configMotorNeutralModes(){
        mAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        mDriveMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference);
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference),
        getAngle());
    }
    
    public SwerveModulePosition getPositionInverted(){
        return new SwerveModulePosition(Conversions.rotationsToMeters(-mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference),
        getAngle());
    }
}
