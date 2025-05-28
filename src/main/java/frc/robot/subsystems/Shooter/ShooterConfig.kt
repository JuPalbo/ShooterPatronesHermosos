package frc.Shooter

import frc.utils.Reduction
import frc.utils.RotationalDirection

data class ShooterConfig(
    val leadMotorControllerId: Int,
    val followerMotorId: Int,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Double,
    val reduction: Reduction,
)
val shooterConfig = ShooterConfig(
    leadMotorControllerId = 1,
    followerMotorId = 1,
    motorDirection = RotationalDirection.Clockwise,
    motorCurrentLimit = 40.0,
    reduction = Reduction(20.0),
)

