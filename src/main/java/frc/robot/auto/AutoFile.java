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
    private final AutoActions autoActions;
    private final Map<String, PathPlannerPath> autoPaths = new HashMap<>();
    
    public AutoFile(AutoActions autoActions) {
        this.autoActions = autoActions;
        initializeAutoPaths();
    }

    private void initializeAutoPaths() {
        File[] files = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();
        assert files != null;
        for (File file : files) {
            try {
                // path files without extension
                PathPlannerPath path = PathPlannerPath.fromPathFile(file.getName());
                autoPaths.put(path.name, path);
            } catch (IOException | ParseException e) {
                throw new IllegalArgumentException("Failed to parse path file: " + file.getName(), e);
            }
        }
    }

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
                autoActions.followPath(getAutoPath("Left"), true, true, false),
                autoActions.shootCoral()
        );
    }
    private Command build1CoralMid() {
        return new SequentialCommandGroup(
                autoActions.followPath(getAutoPath("Mid"), true, true, false),
                autoActions.shootCoral()
        );
    }
    private Command build1CoralRight() {
        return new SequentialCommandGroup(
                autoActions.followPath(getAutoPath("Right"), true, true, false),
                autoActions.shootCoral()
        );
    }
}
