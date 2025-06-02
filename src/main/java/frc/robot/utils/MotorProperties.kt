package frc.robot.utils


data class MotorProperties(

    val positiveDirection: RotationalDirection
)

object Motors {
    /**
     * It holds the default positive direction of a NEO Brushless Motor V1.1
     */
    val neo = MotorProperties(RotationalDirection.CounterClockwise)
}

