package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
public class ElevatorSubsystem extends SubsystemBase {
    public enum ElevatorPosition {
        CORAL_L1, CORAL_L2, CORAL_L3, CORAL_STATION_AND_PROCESSOR, ALGAE_HIGH, ALGAE_LOW, UNKNOWN
    }

    /** TalonFX leader motor controller object */
    private final TalonFX m_leaderMotor;

    /** TalonFX follower motor controller object */
    private final TalonFX m_followerMotor;

    /** Configuration object for the TalonFX motor */
    private final TalonFXConfiguration m_motorConfig;

    /** Voltage control request object for the TalonFX motor controller */
    private final VoltageOut m_voltReq;

    /** Motion magic voltage control request object for the TalonFX motor controller */
    private MotionMagicVoltage m_request;

    /** Object of the Phoenix Orchestra */
    private Orchestra m_orchestra;
    /** Object of a simulated elevator */
    private final ElevatorSim m_elevatorSim;

    /** Mechanism2d object of an elevator */
    private final Mechanism2d m_mech2d;
    /** MechanismRoot2d object of an elevator */
    private final MechanismRoot2d m_mech2dRoot;
    /** MechanismLigament2d object of an elevator */
    private MechanismLigament2d m_elevatorMech2d;

    /** Object of a system identification routine */
    private final SysIdRoutine m_sysIdRoutine;

    /** Target setpoint for the elevator in meters */
    private double m_setpoint;

    /** Indicates whether the elevator has been homed */
    private boolean m_homed;

    /** Array of songs to be played by the Phoenix Orchestra */
    private String[] m_songs = new String[] {"song1.chrp", "song2.chrp"};

    /** Creates a new ElevatorSubsystem */
    public ElevatorSubsystem() {
        if (RobotBase.isSimulation()) {
            //SmartDashboard.putData("Elevator Sim", m_mech2d); commented out for testing
            m_elevatorMech2d.setColor(new Color8Bit(Color.kAntiqueWhite));
        }

        m_leaderMotor = new TalonFX(ElevatorConstants.kElevatorLeaderCAN);

        /** TalonFX follower motor controller object */
        m_followerMotor = new TalonFX(ElevatorConstants.kElevatorFollowerCAN);
    
        /** Configuration object for the TalonFX motor */
        m_motorConfig = new TalonFXConfiguration();
    
        /** Voltage control request object for the TalonFX motor controller */
        m_voltReq = new VoltageOut(0.0);
    
        /** Motion magic voltage control request object for the TalonFX motor controller */
        m_request = new MotionMagicVoltage(0).withSlot(0);
    
        /** Object of the Phoenix Orchestra */
        m_orchestra = new Orchestra();
    
        /** Object of a simulated elevator */
        m_elevatorSim = new ElevatorSim(DCMotor.getFalcon500(2),
                ElevatorConstants.kElevatorGearing, ElevatorConstants.kElevatorCarriageMass,
                ElevatorConstants.kElevatorDrumRadius, ElevatorConstants.kElevatorMinHeightMeters,
                ElevatorConstants.kElevatorMaxHeightMeters, true,
                ElevatorConstants.kElevatorHeightMeters, 0.01, // add some noise
                0);
    
        /** Mechanism2d object of an elevator */
        m_mech2d =
                new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(50));
        /** MechanismRoot2d object of an elevator */
        m_mech2dRoot =
                m_mech2d.getRoot("Elevator Root", Units.inchesToMeters(5), Units.inchesToMeters(0.5));
        /** MechanismLigament2d object of an elevator */
        m_elevatorMech2d =
                m_mech2dRoot.append(new MechanismLigament2d("Elevator",
                        m_elevatorSim.getPositionMeters(), 90, 7, new Color8Bit(Color.kAntiqueWhite)));
    
        /** Object of a system identification routine */
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_leaderMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null, this));

        // Set motor configuration
        m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set slot 0 values
        m_motorConfig.Slot0.kP = ElevatorConstants.kElevatorP;
        m_motorConfig.Slot0.kI = ElevatorConstants.kElevatorI;
        m_motorConfig.Slot0.kD = ElevatorConstants.kElevatorD;
        m_motorConfig.Slot0.kS = ElevatorConstants.kElevatorS;
        m_motorConfig.Slot0.kV = ElevatorConstants.kElevatorV;
        m_motorConfig.Slot0.kA = ElevatorConstants.kElevatorA;
        m_motorConfig.Slot0.kG = ElevatorConstants.kElevatorG;

        // Set gravity type
        m_motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Set motion magic
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity =
                ElevatorConstants.kElevatorMaxVelocity;
        m_motorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorMaxAcceleration;

        // Set safety limits
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                ElevatorConstants.kElevatorMaxHeightMeters
                        / ElevatorConstants.kElevatorMetersPerMotorRotation;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        // Set current limits
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = 125;

        // Set current limits
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = 60;

        // Set follower
        m_followerMotor.setControl(new Follower(m_leaderMotor.getDeviceID(), true));

        // Apply configuration to motors
        m_leaderMotor.getConfigurator().apply(m_motorConfig);
        m_followerMotor.getConfigurator().apply(m_motorConfig);

        // Set update frequencies
        m_leaderMotor.getPosition().setUpdateFrequency(50);
        m_leaderMotor.getVelocity().setUpdateFrequency(50);
        m_leaderMotor.getDutyCycle().setUpdateFrequency(50);
        m_leaderMotor.getMotorVoltage().setUpdateFrequency(50);
        m_leaderMotor.getTorqueCurrent().setUpdateFrequency(50);

        m_orchestra.addInstrument(m_leaderMotor);
        m_orchestra.addInstrument(m_followerMotor);

        m_homed = false;
    }
    public double getPosition() {
        return m_leaderMotor.getPosition().getValueAsDouble();
    }
        public double getPositionMeters() {
        return getPosition() * ElevatorConstants.kElevatorMetersPerMotorRotation;
    }
    public double getVelocityMetersPerSecond() {
        return m_leaderMotor.getVelocity().getValueAsDouble()* ElevatorConstants.kElevatorMetersPerMotorRotation;
    }
    public void zero() {
        m_leaderMotor.setPosition(0);
    }

    public boolean atHeight() {
        return Math.abs(getPositionMeters() - m_setpoint) < ElevatorConstants.kElevatorTargetError;
    }

    
    public void stop() {
        m_leaderMotor.stopMotor();
        m_followerMotor.stopMotor();
    }

    
    public void updateTelemetry() {
        m_elevatorMech2d.setLength(getPositionMeters());
    }

    /**
     * Set the position of the elevator using Motion Magic control
     * 
     * @param position The position in meters
     */
    public void setPosition(double position) {

        if (position > ElevatorConstants.kElevatorMaxHeightMeters) {
            m_setpoint = ElevatorConstants.kElevatorMaxHeightMeters;
        } else if (position < ElevatorConstants.kElevatorMinHeightMeters) {
            m_setpoint = ElevatorConstants.kElevatorMinHeightMeters;
        } else {
            m_setpoint = position;
        }

        double positionRotations = position / ElevatorConstants.kElevatorMetersPerMotorRotation;
        m_request = m_request.withPosition(positionRotations).withSlot(0);
        m_leaderMotor.setControl(m_request);
    }

    /**
     * Returns whether the elevator is at the retract limit
     * 
     * @return Whether the elevator is at the retract limit
     */
    public boolean isAtRetractLimit() {
        return m_leaderMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    /**
     * Returns a command that will execute a quasistatic test in the given direction
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    
    public ElevatorPosition getElevatorEnumPosition() {
        
        double currentPosition = getPositionMeters();

        if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorCoralLevel1StartHeight) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.CORAL_L1;
        } else if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorCoralLevel2Height) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.CORAL_L2;
        } else if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorCoralLevel3Height) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.CORAL_L3;
        } else if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorCoralStationAndProcessorHeight) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.CORAL_STATION_AND_PROCESSOR;
        } else if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorAlgaeHighHeight) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.ALGAE_HIGH;
        } else if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorAlgaeLowHeight) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.ALGAE_LOW;
        } else {
            return ElevatorPosition.UNKNOWN;
        }
    }

    @Override
    public void periodic() {
        if (m_leaderMotor.getClosedLoopReference().getValueAsDouble() == 0
                && m_leaderMotor.getPosition().getValueAsDouble() < 0.5) {
            m_leaderMotor.setVoltage(0);

        } else {
            m_leaderMotor.setControl(m_request);

        }

        updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.setInputVoltage(m_leaderMotor.getMotorVoltage().getValueAsDouble());
        m_elevatorSim.update(0.020);

        final double positionRot = m_elevatorSim.getPositionMeters()
                / ElevatorConstants.kElevatorMetersPerMotorRotation;
        final double velocityRps = m_elevatorSim.getVelocityMetersPerSecond()
                / ElevatorConstants.kElevatorMetersPerMotorRotation;

        m_leaderMotor.getSimState().setRawRotorPosition(positionRot);
        m_leaderMotor.getSimState().setRotorVelocity(velocityRps);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

}