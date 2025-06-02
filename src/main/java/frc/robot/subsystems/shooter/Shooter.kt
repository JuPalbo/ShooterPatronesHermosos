package frc.robot.subsystems.shooter

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.Shooter.ShooterConfig

class Shooter(private val config: ShooterConfig) : SubsystemBase() {
    private val leadMotorController = SparkMax(config.leadMotorControllerId, SparkLowLevel.MotorType.kBrushless)
    private val followerMotorController = SparkMax(config.followerMotorId, SparkLowLevel.MotorType.kBrushless)

    /**
     * This function accepts voltage and passes it to the motor controller within a safe
     * range for the robot
     * @param voltage the desired voltage for the motor
     */
    fun setVoltage(voltage: Voltage) {
        val clampedVoltage = MathUtil.clamp(voltage.`in`(Units.Volts), config.shootVoltageLowLimit, config.shootVoltageHighLimit)
        leadMotorController.setVoltage(clampedVoltage)
    }

    /**
     * This function is able to set the motor's voltage to 0 and therefore, stop the motor's
     * interaction
     */
    fun stopMotors() : Command = Commands.run({setVoltage(Units.Volts.of(0.0))})

    /**
     * The outTakeCMD Command is able to set a desired voltage to the motors when
     * the command is firstly called and when interrupted, it will completely
     * stop the motors.
     * @param voltage the desired voltage to set to the motors
     * @return a command that sets a voltage to the subsystem's motors.
     */
    fun shootCMD(voltage: Voltage) : Command = Commands.startEnd(
        { setVoltage(voltage) },
        { stopMotors() },
        this)

    init {
        configureMotorsInterface()
    }

    /**
     * Configures the motors' Idle Mode, positive direction, current limit and clears Spark's
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

