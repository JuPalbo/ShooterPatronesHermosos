package frc.robot.subsystems.shooter

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj2.command.SubsystemBase

import frc.robot.subsystems.shooter.ShooterState.*
import frc.robot.utils.opposite

/**
 * The [ShooterState] enum class creates two states that the shooter can be on, either [ButtonMode]
 * or [TriggerMode].
 */
enum class ShooterState {
    TriggerMode,
    ButtonMode
}

class Shooter(private val config: ShooterConfig) : SubsystemBase() {
    private val leadMotorController = SparkMax(config.leadMotorControllerId.id, SparkLowLevel.MotorType.kBrushless)
    private val followerMotorController = SparkMax(config.followerMotorId.id, SparkLowLevel.MotorType.kBrushless)

    /**
     * Tracks the voltage that is being given to the motors and passes it to [periodic]
     */
    private var voltageOutPut = Volts.of(0.0)

    /**
     * Part of the [ShooterState] enum class. Gives a default [ShooterState] value to a variable for it to
     * be manageable within the code
     */
    private var currentShooterState = TriggerMode

    /**
     * Constantly checks if any voltage has been given to the motors in order to update and set it
     * immediately, then, clamps it to prevent excessive voltage to be given to the motors.
     * @return a new voltage to set to the motor
     */
    override fun periodic() {
        val clampedVoltage =
            MathUtil.clamp(voltageOutPut.`in`(Volts)
                , shooterConfig.shootVoltageLowLimit.`in`(Volts)
                , shooterConfig.shootVoltageHighLimit.`in`(Volts))
        leadMotorController.setVoltage(clampedVoltage)
    }

    /**
     * The currentState() function returns the [ShooterState] that is currently applied to this subsystem
     * @return the current shooter state, either button or trigger state's
     */
    @JvmName("ShooterCurrentState")
    fun currentState(): ShooterState = currentShooterState

    /**
     * The changeState() uses the [ShooterState] class to change the current state to it's opposite
     * @return a changed state to the shooter
     */
    fun changeState() {
        currentShooterState =
            if (currentState() == ButtonMode) TriggerMode
            else ButtonMode
    }

    /**
     * This function accepts voltage and passes it to the motor controller through [voltageOutPut]
     * @param voltage the desired voltage for the motor
     */
    fun setVoltage(voltage: Voltage) {
        voltageOutPut = voltage
    }

    /**
     * Adds, to the current [voltageOutPut], the desired volts
     * @param voltage the desired voltage to add
     * @return returns a new [voltageOutPut] value that is set to the motor plus one
     */
    fun addVolts(voltage: Voltage) {
        voltageOutPut = voltageOutPut.plus(voltage)
    }

    /**
     * Subtracts, to the current [voltageOutPut], the desired volts
     * @param voltage the desired voltage to subtract
     * @return returns a new [voltageOutPut] value that is set to the motor minus one
     */
    fun subtractsVolts(voltage: Voltage) {
        voltageOutPut = voltageOutPut.minus(voltage)
    }

    /**
     * This function is able to set the motor's [voltageOutPut] to 0 and therefore, stop the motor's
     * interaction
     */
    fun stopMotors() {
        voltageOutPut = Volts.of(0.0)
    }

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

        //Setting an idle mode, giving an inverted value and establishing a current limit to the motor's
        with(globalConfig) {
            idleMode(config.motorProperties.neutralMode).inverted(
                config.motorDirection.opposite() == config.motorProperties.positiveDirection
            ).smartCurrentLimit(config.motorProperties.currentLimit.`in`(Amps).toInt())
        }

        //Apply a follow method to set both motors at the same time
        followerConfig.apply(globalConfig).follow(config.leadMotorControllerId.id, true) // Might change inverted

        //Clear the Spark´s faults
        leadMotorController.clearFaults()
        followerMotorController.clearFaults()

        //Set the lead motor´s desired configurations
        leadMotorController.configure(
            globalConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters
        )

        //Set the follower motor's desired configurations
        followerMotorController.configure(
            followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters
        )
    }
}

