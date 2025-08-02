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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.units.measure.*;

public class ArmSubsystem extends SubsystemBase{
    // Declare feedforward system
    private final ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS,ArmConstants.kG,ArmConstants.kV,ArmConstants.kA);
    //Declares Motorctrl Var
    private final TalonFX motor = new TalonFX(ArmConstants.canID);
    private final TalonFXSimState motorSim;
    private final PositionVoltage posReq;
    private final VelocityVoltage velReq;
    private final StatusSignal<Angle> posSignal;
    private final StatusSignal<AngularVelocity> velSignal;
    private final StatusSignal<Voltage> volsignal;
    private final StatusSignal<Current> currSignal;
    private final StatusSignal<Temperature> tempSignal;

    //Declares simulation variables
    private final SingleJointedArmSim armSim;
    private final Mechanism2d mech2d = new Mechanism2d(1f,1f);
    private final MechanismRoot2d root = mech2d.getRoot("Arm",ArmConstants.armLength,90);
    private final MechanismLigament2d armLig = root.append(new MechanismLigament2d("Arm", ArmConstants.armLength, 0));
    private final MotionMagicVoltage armMotionMagicControl;

    //Constructor
    public ArmSubsystem() {
        //modelled after original arm repo
        motorSim = motor.getSimState();
        posReq=new PositionVoltage(0).withSlot(0);
        velReq=new VelocityVoltage(0).withSlot(0);
        posSignal = motor.getPosition();
        velSignal=motor.getVelocity();
        volsignal = motor.getMotorVoltage();
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
    }
}
