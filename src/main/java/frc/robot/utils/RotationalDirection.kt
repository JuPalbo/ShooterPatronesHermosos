package frc.utils

import com.ctre.phoenix6.signals.InvertedValue.*

/**
 * This class is able to set the rotational direction to either clockwise or
 * counterclockwise to a TalonFX
 *@property factor this will be either clockwise or counterclockwise
 */
enum class RotationalDirection(private val factor: Int) {
    CounterClockwise(1), Clockwise(-1);

    /**
     * The function sets the positive rotational direction to clockwise or counterclockwise
     * @return returns the positive direction for the motor if it needs to be either clockwise or counterclockwise
     */
    fun toInvertedValue() = if (this == Clockwise) Clockwise_Positive else CounterClockwise_Positive
}
