package frc.robot.commands.autocmd;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;

public class DrivePathCommand {
    public static Command drivePath(String pathToFollow) {
        System.out.println(pathToFollow);
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathToFollow);
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }
}
