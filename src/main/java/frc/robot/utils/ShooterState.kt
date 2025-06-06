package frc.robot.utils

enum class ShooterState(private val factor: Int) {
    ButtonMode(1), TriggerMode(-1);

    fun changeMode()= if (this == ButtonMode) TriggerMode else ButtonMode
}