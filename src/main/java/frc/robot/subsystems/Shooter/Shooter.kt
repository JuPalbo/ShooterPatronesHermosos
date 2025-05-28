package frc.Shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import edu.wpi.first.units.measure.Voltage.*


import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.utils.RotationalDirection

class Shooter(private val config: ShooterConfig) : SubsystemBase() {
    private val leadMotorController = TalonFX(config.leadMotorControllerId)
    private val followerMotorController = TalonFX(config.followerMotorId)

    val ForwardsRunningCondition = { true }
    val BackwardsRunningCondition = { true }

    /**
     * This function accepts voltage and passes it to the motor controller
     * @param voltage the desired voltage for the motor
     * @return A voltage to the motor controller
     */
    private fun setVoltage(voltage: Voltage) {
        val voltageRequest = VoltageOut(voltage)
        leadMotorController.setControl(voltageRequest)
    }

    fun stopMotors() : Command = Commands.run({setVoltage(Units.Volts.of(0.0))})

    fun setVoltageCMD(voltage: Voltage) : Command = Commands.run({ setVoltage(voltage) })

    fun shootCMD(voltage: Voltage) : Command = Commands.startEnd(
        { setVoltage(voltage) },
        { stopMotors() },
        this)

    val motorVelocity: AngularVelocity
        get() = leadMotorController.velocity.value
    val power: Double
        get() = leadMotorController.get()

    init {
        configureMotorsInterface()

    }

    private fun configureMotorsInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.motorDirection.toInvertedValue())

            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorCurrentLimit)

        }

        leadMotorController.clearStickyFaults()
        followerMotorController.clearStickyFaults()

        leadMotorController.configurator.apply(talonConfig)
        followerMotorController.configurator.apply(talonConfig)

        followerMotorController.setControl(Follower(leadMotorController.deviceID, true))
    }
}

