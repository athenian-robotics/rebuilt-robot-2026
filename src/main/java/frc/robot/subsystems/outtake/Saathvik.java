package frc.robot.subsystems.outtake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;

public class Saathvik implements BooleanSupplier, DoubleSupplier {
    public static void projectCadOnScreeen() {}
    public static String drawTree() {return "Robot\nDrive|Hopintake|Indexer|Output\nHopper|Intake|Reversible stealth wheels|Hood|All other output motors";}
    public double mechanics(Pose2d pos) {return getAsDouble();}

    @Override
    public double getAsDouble() {
      return 0.0;
    }
    @Override
    public boolean getAsBoolean() {
      return true;
    }
}
