package frc.robot.auto;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoFile {
    private final AutoActions autoActions = new AutoActions();
    private final Map<String, PathPlannerPath> autoPaths = new HashMap<>();
    
    private PathPlannerPath getAutoPath(String path) {
        assert autoPaths.containsKey(path);
        return autoPaths.get(path);
    }

    public Command runAuto(String autoName) {
        return switch (autoName) {
            case "1CoralLeft" -> build1CoralLeft();
            case "1CoralMid" -> build1CoralMid();
            case "1CoralRight" -> build1CoralRight();
            default -> throw new IllegalArgumentException("No corresponding auto named " + autoName);
        };
    }

    private Command build1CoralLeft() {
        return new SequentialCommandGroup(
                autoActions.followPath("Left"),
                autoActions.shootCoral()
        );
    }
    private Command build1CoralMid() {
        return new SequentialCommandGroup(
                autoActions.followPath("Mid"),
                autoActions.shootCoral()
        );
    }
    private Command build1CoralRight() {
        return new SequentialCommandGroup(
                autoActions.followPath("Right"),
                autoActions.shootCoral()
        );
    }
}
