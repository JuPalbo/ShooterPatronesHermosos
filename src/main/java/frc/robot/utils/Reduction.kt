package frc.robot.utils

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit


/**
 * This class is able to apply and un-apply the reduction to the motor's and subsystem's output
 * so their values are better manageable
 * @property ratio is the subsystem's ratio. For example 40 : 2 = 20
 */
class Reduction(private val ratio: Double) {
    /**
     * This function applies the system's reduction to a value, which is the rotations
     * of the motor in any unit measure, so that we can have the full subsystem
     * rotations.
     *
     * @param value this is the value is the motor's output
     *
     * @return returns full subsystem's output, taking into account
     * the motor's output and the reduction
     */
    fun <Medidas : Measure<out Unit>> apply(value: Medidas) = value.div(ratio) as Medidas

    /**
     * This function un-applies the reduction to the full subsystem's output value
     * so it the motor's value can be used
     * @param value this is the subsystem value
     * @return returns the motor´s output, taking into account the full subsystem's
     * output and the system reduction
     */
    fun <Medidas : Measure<out Unit>> unapply(value: Medidas) = value.times(ratio) as Medidas
}