package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {
    protected final RollerIO io;
    protected final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    private final String name;

    public RollerSubsystem(RollerIO io, String name) {
        this.io = io;
        this.name = name;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }
}
