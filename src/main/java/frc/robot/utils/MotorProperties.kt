package frc.robot.utils

import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import frc.robot.utils.RotationalDirection.CounterClockwise

/**
 * It is useful to keep in a single place various configurations for a single motor such as a
 * [RotationalDirection] , an [IdleMode] and a [Current] limit value.
 */
data class MotorProperties(

    val positiveDirection: RotationalDirection,
    val neutralMode: IdleMode ,
    val currentLimit: Current,

)

object Motors {
    /**
     * It holds the default positive direction, desired idle mode and motor's current limit
     * of a NEO Brushless Motor V1.1
     */
    val neo = MotorProperties(CounterClockwise, IdleMode.kBrake, Units.Amps.of(40.0))
    //val kraken = MotorProperties(CounterClockwise, NeutralModeValue.Brake, Units.Amps.of(40.0))
}

