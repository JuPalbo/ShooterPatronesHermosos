package frc.robot.subsystems.shooter

import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import frc.robot.utils.*
import frc.robot.utils.RotationalDirection.Clockwise

/**
 * Useful for having a much better understanding on ID´s
 * @property id The ID of the motor, this must be an integer value
 */
@JvmInline
value class NumericID(val id: Int) {
    init {
        require(id > 0) { "$id should be bigger than zero." }
    }
}

/**
 * Useful to keep all the desired shooter configurations and it's types to later assign them to a value
 */
data class ShooterConfig(
    val motorProperties: MotorProperties,
    val leadMotorControllerId: NumericID,
    val followerMotorId: NumericID,
    val motorDirection: RotationalDirection,
    val voltageLowLimit: Voltage,
    val voltageHighLimit: Voltage
)
val primaryShooterConfig = ShooterConfig(
    motorProperties = Motors.neo,
    leadMotorControllerId = NumericID(1),
    followerMotorId = NumericID(31),
    motorDirection = Clockwise, // Check the actual direction ,
    voltageLowLimit = Volts.of(-12.0),
    voltageHighLimit = Volts.of(12.0)
)

val secondaryShooterConfig = ShooterConfig(
    motorProperties = Motors.neo,
    leadMotorControllerId = NumericID(7),
    followerMotorId = NumericID(9),
    motorDirection = Clockwise, // Check the actual direction ,
    voltageLowLimit = Volts.of(-12.0),
    voltageHighLimit = Volts.of(12.0)
)

