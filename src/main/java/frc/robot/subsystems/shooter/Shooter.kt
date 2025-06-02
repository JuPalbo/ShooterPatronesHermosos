package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.controls.Follower

import com.revrobotics.*
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig

import edu.wpi.first.units.measure.Voltage.*
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.Shooter.ShooterConfig

class Shooter(private val config: ShooterConfig) : SubsystemBase() {
    private val leadMotorController = SparkMax(config.leadMotorControllerId, SparkLowLevel.MotorType.kBrushless)
    private val followerMotorController = SparkMax(config.followerMotorId, SparkLowLevel.MotorType.kBrushless)

    val ForwardsRunningCondition = { true }
    val BackwardsRunningCondition = { true }

    /**
     * This function accepts voltage and passes it to the motor controller
     * @param voltage the desired voltage for the motor
     * @return A voltage to the motor controller
     */
    private fun setVoltage(voltage: Voltage) {
        val voltageRequest = VoltageOut(voltage)
        leadMotorController.setVoltage(voltage)
    }

    /**
     * This function is able to set the motor's voltage to 0 and therefore, stop the motor's
     * interaction
     * @return A previously set voltage for the motors, in this case it would be 0.
     */
    fun stopMotors() : Command = Commands.run({setVoltage(Units.Volts.of(0.0))})

    /**
     * The outTakeCMD Command is able to set a desired voltage to the motors when
     * the command is firstly called and when interrupted, it will completely
     * stop the motors.
     * @param voltage the desired voltage to set to the motors
     * @return a command that sets a voltage to the subsystem's motors.
     */
    fun outTakeCMD(voltage: Voltage) : Command = Commands.startEnd(
        { setVoltage(voltage) },
        { stopMotors() },
        this)

    init {
        configureMotorsInterface()
    }

    /**
     * Configures the motors' Idle Mode, positive direction, current limit and clears Spark's
     * faults.
     * @return the SparkMax desired configuration
     */
    private fun configureMotorsInterface() {
        val sparkMaxConfig = SparkMaxConfig()

        with(sparkMaxConfig) {
            idleMode(IdleMode.kBrake).inverted(
                config.motorDirection.opposite() == config.motorProperties.positiveDirection
            ).smartCurrentLimit(config.motorCurrentLimit.`in`(Units.Amps).toInt())
        }

        leadMotorController.clearFaults()
        followerMotorController.clearFaults()

        leadMotorController.configure(
            sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters
        )
        followerMotorController.configure(
            sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters
        )

    }
}

