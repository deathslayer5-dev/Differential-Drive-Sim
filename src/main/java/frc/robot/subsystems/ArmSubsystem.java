package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.units.measure.*;

public class ArmSubsystem extends SubsystemBase{
    /*psuedo code
    declare feedforward system
    declare motor control variables
    declare variables for simulation
    make constructor
    define class variables
    define parameters
    init arm sim
    make getters and setters{
        call get function: return value
        call set function, given paramter: value = parameter
    }
    make commands for arm control{
        periodic: executes every scheduler run
        simulationPeriodic: executes every scheduler run in simulation
    }
    */

    // Declare feedforward system
    private final ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS,ArmConstants.kG,ArmConstants.kV,ArmConstants.kA);
    //Declares Motorctrl Var
    private final TalonFX motor = new TalonFX(ArmConstants.canID);
    private final TalonFXSimState motorSim;
    private final PositionVoltage posReq;
    private final VelocityVoltage velReq;
    private final StatusSignal<Angle> posSignal;
    private final StatusSignal<AngularVelocity> velSignal;
    private final StatusSignal<Voltage> volSignal;
    private final StatusSignal<Current> currSignal;
    private final StatusSignal<Temperature> tempSignal;

    //Declares simulation variables
    private final SingleJointedArmSim armSim;
    private final Mechanism2d mech2d = new Mechanism2d(1f,1f);
    private final MechanismRoot2d root = mech2d.getRoot("Arm root",0.5,0.1);
    private final MechanismLigament2d armLig = root.append(new MechanismLigament2d("Arm", ArmConstants.armLength, 90));
    private final MotionMagicVoltage armMotionMagicControl;

    //Constructor
    public ArmSubsystem() {
        //modelled after original arm repo
        motorSim = motor.getSimState();
        posReq=new PositionVoltage(0).withSlot(0);
        velReq=new VelocityVoltage(0).withSlot(0);
        posSignal = motor.getPosition();
        velSignal=motor.getVelocity();
        volSignal = motor.getMotorVoltage();
        currSignal = motor.getStatorCurrent();
        tempSignal = motor.getDeviceTemp();

        armMotionMagicControl = new MotionMagicVoltage(0);

        //parameters
        TalonFXConfiguration config = new TalonFXConfiguration();
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = ArmConstants.kP;
        slot0.kI = ArmConstants.kI;
        slot0.kD = ArmConstants.kD;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = ArmConstants.statorLimit;
        currentLimits.StatorCurrentLimitEnable = ArmConstants.enableStatorLimit;
        currentLimits.SupplyCurrentLimit = ArmConstants.supplyLimit;
        currentLimits.SupplyCurrentLimitEnable = ArmConstants.enableSupplyLimit;

        config.Feedback.SensorToMechanismRatio = ArmConstants.gearRatio;

        config.MotionMagic.MotionMagicCruiseVelocity = 10f;
        config.MotionMagic.MotionMagicAcceleration = 20f;
        config.MotorOutput.NeutralMode=ArmConstants.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        motor.getConfigurator().apply(config);
        motor.setPosition(0);
        //init
        armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ArmConstants.gearRatio,
            SingleJointedArmSim.estimateMOI(ArmConstants.armLength, 5),
            ArmConstants.armLength,
            Units.degreesToRadians(ArmConstants.minAngleDeg),
            Units.degreesToRadians(ArmConstants.maxAngleDeg),
            true,
            Units.degreesToRadians(0)
        );
        armLig.setLength(ArmConstants.armLength);
        SmartDashboard.putData("Arm Visualization",mech2d);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        BaseStatusSignal.refreshAll(posSignal,
                                    velSignal,
                                    volSignal,
                                    currSignal,
                                    tempSignal);
        SmartDashboard.putNumber("Arm/Position(rot)",motor.getPosition().getValueAsDouble());

        //SmartDashBoard inputting parameters
        SmartDashboard.putNumber("Arm/Velocity(rps)",getVelocity());
        SmartDashboard.putNumber("Arm/Voltage",getVoltage());
        SmartDashboard.putNumber("Arm/Current",getCurrent());
        SmartDashboard.putNumber("Arm/Temperature",getTemperature());
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("Sim/TargetRotations", armMotionMagicControl.Position);
        SmartDashboard.putNumber("Sim/SimAngleDeg", Units.radiansToDegrees(armSim.getAngleRads()));
        SmartDashboard.putNumber("Sim/MotorVoltage", motorSim.getMotorVoltage());

        motorSim.setSupplyVoltage(12);

        armSim.setInput(motorSim.getMotorVoltage());
        armSim.update(0.02);

        motorSim.setRawRotorPosition(
            (armSim.getAngleRads() - Math.toRadians(ArmConstants.minAngleDeg))
                * ArmConstants.gearRatio
                / (2.0
                * Math.PI));

        motorSim.setRotorVelocity(
        armSim.getVelocityRadPerSec() * ArmConstants.gearRatio / (2.0 * Math.PI));

        armLig.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }
    public double getPosition(){
        return posSignal.getValueAsDouble();
    }
    public double getVelocity(){
        return velSignal.getValueAsDouble();
    }
    public double getVoltage(){
        return volSignal.getValueAsDouble();
    }
    public double getCurrent(){
        return currSignal.getValueAsDouble();
    }
    public double getTemperature(){
        return tempSignal.getValueAsDouble();
    }
    public double getPositionRadians(){
        return getPosition() *2f*Math.PI;
    }
    public void setPosition(double position){
        double rotations = Math.toRadians(position) / (2f*Math.PI);
        armMotionMagicControl.Slot=0;
        armMotionMagicControl.Position=rotations;
        motor.setControl(armMotionMagicControl);
    }
    public void setVelocity(double velocityDegPerSec) {
        setVelocity(velocityDegPerSec, 0);
      }
    
       // Method to set the arm velocity with specified acceleration (in degrees per second)
      public void setVelocity(double velocityDegPerSec, double acceleration) {
        double currentDeg = Units.radiansToDegrees(getPositionRadians());
        if ((currentDeg >= ArmConstants.maxAngleDeg && velocityDegPerSec > 0) ||
            (currentDeg <= ArmConstants.minAngleDeg && velocityDegPerSec < 0)) {
          velocityDegPerSec = 0;
        }
        double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
        double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);
        double ffVolts = feedforward.calculate(getVelocity(), acceleration);
        motor.setControl(velReq.withVelocity(velocityRotations).withFeedForward(ffVolts));
      }
      public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
      }
    
      public Command setAngleCommand(double angleDegrees) {
        return runOnce(() -> setPosition(angleDegrees));
      }
    
      public Command stopCommand() {
        return runOnce(() -> setVelocity(0));
      }
    
      public Command moveAtVelocityCommand(double velocityDegPerSec) {
        return run(() -> setVelocity(velocityDegPerSec));
    }
}
