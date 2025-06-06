package frc.robot.subsystems.shooter

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

import frc.Shooter.ShooterConfig
import frc.Shooter.shooterConfig
import frc.robot.utils.ShooterState
import frc.robot.utils.ShooterState.TriggerMode
import frc.robot.utils.ShooterState.ButtonMode


class Shooter(private val config: ShooterConfig) : SubsystemBase() {
    private val leadMotorController = SparkMax(config.leadMotorControllerId, SparkLowLevel.MotorType.kBrushless)
    private val followerMotorController = SparkMax(config.followerMotorId, SparkLowLevel.MotorType.kBrushless)
    private var voltageOutPut = Volts.of(0.0)
    private var currentShooterMode = TriggerMode

    override fun periodic() {
        val clampedVoltage = MathUtil.clamp(voltageOutPut.`in`(Volts), shooterConfig.shootVoltageLowLimit, shooterConfig.shootVoltageHighLimit)
        leadMotorController.setVoltage(clampedVoltage)
    }

    fun currentMode() = currentShooterMode

    /**
     * This function accepts voltage and passes it to the motor controller within a safe
     * range for the robot
     * @param voltage the desired voltage for the motor
     */
    fun setVoltage(voltage: Voltage) {
        voltageOutPut = voltage
    }

    fun addVolts(voltage: Voltage) {
        voltageOutPut = voltageOutPut.plus(voltage)
    }

    /**
     * Subtracts from the current voltage output the desired volts
     * @param voltage the desired voltage to subtract
     * @return returns a new voltage value that is set to the motor
     */
    fun subtractsVolts(voltage: Voltage) {
        voltageOutPut = voltageOutPut.minus(voltage)
    }

    /**
     * This function is able to set the motor's voltage to 0 and therefore, stop the motor's
     * interaction
     */
    fun stopMotors() {
        voltageOutPut = Volts.of(0.0)
    }

    /**
     * The outTakeCMD Command is able to set a desired voltage to the motors when
     * the command is firstly called and when interrupted, it will completely
     * stop the motors.
     * @param voltage the desired voltage to set to the motors
     * @return a command that sets a voltage to the subsystem's motors.
     */
    fun shootCMD(voltage: Voltage) : Command = Commands.run(
        { setVoltage(voltage) },
        this)

    init {
        configureMotorsInterface()
    }

    /**
     * Configures the motors' desired sets, such as: Idle Mode, positive direction, current limit and clears Spark's
     * faults.
     */
    private fun configureMotorsInterface() {
        val globalConfig = SparkMaxConfig()
        val followerConfig = SparkMaxConfig()

        with(globalConfig) {
            idleMode(IdleMode.kBrake).inverted(
                config.motorDirection.opposite() == config.motorProperties.positiveDirection
            ).smartCurrentLimit(config.motorCurrentLimit.`in`(Units.Amps).toInt())
        }

        //Apply a follow method to set both motors at the same time
        followerConfig.apply(globalConfig).follow(config.leadMotorControllerId, false) // Might change inverted

        leadMotorController.clearFaults()
        followerMotorController.clearFaults()

        leadMotorController.configure(
            globalConfig, SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters
        )
        followerMotorController.configure(
            followerConfig, SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters
        )
    }
}

