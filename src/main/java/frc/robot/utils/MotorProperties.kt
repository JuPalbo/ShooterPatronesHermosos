package frc.robot.utils

import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import frc.robot.utils.RotationalDirection.Clockwise
import frc.robot.utils.RotationalDirection.CounterClockwise


data class MotorProperties(

    val positiveDirection: RotationalDirection,
    val neutralMode: NeutralModeValue,
    val currentLimit: Current,

)

object Motors {
    /**
     * It holds the default positive direction of a NEO Brushless Motor V1.1
     */
    val neo = MotorProperties(CounterClockwise, NeutralModeValue.Brake, Units.Amps.of(40.0))
    val kraken = MotorProperties(CounterClockwise, NeutralModeValue.Brake, Units.Amps.of(40.0))
}

