package frc.robot.core

import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

import frc.robot.subsystems.shooter.shooterConfig
import frc.robot.core.Constants.OperatorConstants
import frc.robot.commands.Autos
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.shooter.ShooterState

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer
{
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private val driverController = CommandXboxController(OperatorConstants.controllerId)
    private val shooter = Shooter(shooterConfig)

    init
    {
        configureBindings()
        // Reference the Autos object so that it is initialized, placing the chooser on the dashboard
        Autos
    }

    fun teleopInit() {
        /**
         * Sets the default command to either one of the two shooter states and configures it's bindings
         * and associated commands
         */
        shooter.defaultCommand = Commands.run({
            when (shooter.currentState()) {
                 ShooterState.ButtonMode -> {

                    driverController.leftTrigger()
                        .onTrue(InstantCommand({ shooter.setVoltage(Volts.of(-12.0)) }))
                        .onFalse(InstantCommand(shooter::stopMotors))

                    driverController.rightTrigger()
                        .onTrue(InstantCommand({ shooter.setVoltage(Volts.of(12.0)) }))
                        .onFalse(InstantCommand(shooter::stopMotors))
                }

                ShooterState.TriggerMode ->

                    when {
                        driverController.rightTriggerAxis > 0.1 ->
                            shooter.setVoltage(Volts.of(10.0.times(driverController.rightTriggerAxis)))
                        driverController.leftTriggerAxis > 0.1 ->
                            shooter.setVoltage(Volts.of((-10.0).times(driverController.leftTriggerAxis)))
                        else -> shooter.stopMotors()
                    }
                }
            }, shooter
        )
    }


    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier]
     * with an arbitrary predicate, or via the named factories in [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID]
     * subclasses such for [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        //Trigger { ExampleSubsystem.exampleCondition() }.onTrue(ExampleCommand())

        //Adds one volt to the current voltage output
        driverController.b().onTrue(InstantCommand({ shooter.addVolts(Volts.of(1.0)) }))

        //Subtracts one volt from the current voltage output
        driverController.a().onTrue(InstantCommand({ shooter.subtractsVolts(Volts.of(1.0)) }))

        //Changes from Button State to Trigger State and vice versa
        driverController.x().onTrue(InstantCommand({ shooter.changeState()}))

        //Prints the current shooter State so the user knows which bindings are available
        driverController.y().onTrue(InstantCommand({ println(shooter.currentState().toString())}))
        }
    }
