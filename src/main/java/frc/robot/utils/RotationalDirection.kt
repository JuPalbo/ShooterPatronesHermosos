package frc.robot.utils

import com.ctre.phoenix6.signals.InvertedValue.*
import frc.robot.utils.RotationalDirection.CounterClockwise
import frc.robot.utils.RotationalDirection.Clockwise

/**
 * This class is able to set the rotational direction to either clockwise or
 * counterclockwise. It keeps CounterClockwise as 1 and Clockwise as -1.
 *@property factor this will be either clockwise or counterclockwise
 */
enum class RotationalDirection(private val factor: Int) {
    CounterClockwise(1), Clockwise(-1);

    /**
     * The isClockwise() function allows you to check if the value that's passed to
     * it through the Rotational Direction class is Clockwise or not
     * @return Returns true if "factor" is Clockwise or false if it's not
     */
    fun isClockwise(): Boolean = this == Clockwise

    /**
     * The isCounterClockwise() function allows you to check if the value that is passed to it through
     * the Rotational Direction class is Counterclockwise or not
     * @return Returns true if "factor" is Counterclockwise or false if it's not
     */
    fun isCounterClockwise(): Boolean = !isClockwise()

    /**
     * The opposite() function returns the opposite to either Clockwise or Counterclockwise
     * @return the opposite of Clockwise or Counterclockwise
     */
    //fun opposite(): RotationalDirection = if (isCounterClockwise()) Clockwise else CounterClockwise
    /**
     * The function sets the positive rotational direction to clockwise or counterclockwise
     * @return returns the positive direction for the motor if it needs to be either clockwise or counterclockwise
     */
    fun toInvertedValue() = if (this == Clockwise) Clockwise_Positive else CounterClockwise_Positive

}

fun RotationalDirection.opposite(): RotationalDirection = if (isClockwise()) Clockwise else CounterClockwise