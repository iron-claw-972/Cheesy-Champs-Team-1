package lib;


import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.AutoConstants;

public class PathLoader {
  private static HashMap<String, ArrayList<PathPlannerTrajectory>> pathGroups = new HashMap<String, ArrayList<PathPlannerTrajectory>>();

  public static void loadPathGroups() {
    double totalTime = 0;
    File[] directoryListing = Filesystem.getDeployDirectory().toPath().resolve(AutoConstants.kTrajectoryDirectory).toFile().listFiles();
    if (directoryListing != null) {
      for (File file : directoryListing) {
        if (file.isFile() && file.getName().indexOf(".") != -1) {
          long startTime = System.nanoTime();
          String name = file.getName().substring(0, file.getName().indexOf("."));
          pathGroups.put(name, PathPlanner.loadPathGroup(name, new PathConstraints(AutoConstants.kMaxAutoSpeed, AutoConstants.kMaxAutoAccel)));
          double time = (System.nanoTime() - startTime) / 1000000.0;
          totalTime += time;
          System.out.println("Processed file: " + file.getName() + ", took " + time + " milliseconds.");
        }
      }
    } else {
      DriverStation.reportWarning(
        "Issue with finding path files. Paths will not be loaded.",
        true
      );
    }
    System.out.println("File processing took a total of " + totalTime + " milliseconds");
  }

  public static ArrayList<PathPlannerTrajectory> getPathGroup(String pathGroupName) {
    return pathGroups.get(pathGroupName);
  }

}