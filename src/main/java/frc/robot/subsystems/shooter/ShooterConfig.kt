package frc.Shooter

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.robot.utils.MotorProperties
import frc.robot.utils.Motors
import frc.robot.utils.Reduction
import frc.robot.utils.RotationalDirection
import frc.robot.utils.RotationalDirection.Clockwise


data class ShooterConfig(
    val motorProperties: MotorProperties,
    val leadMotorControllerId: Int,
    val followerMotorId: Int,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,
    val reduction: Reduction,
)
val shooterConfig = ShooterConfig(
    motorProperties = Motors.neo,
    leadMotorControllerId = 1,
    followerMotorId = 1,
    motorDirection = Clockwise,
    motorCurrentLimit = Units.Amp.of(40.0),
    reduction = Reduction(20.0),
)

