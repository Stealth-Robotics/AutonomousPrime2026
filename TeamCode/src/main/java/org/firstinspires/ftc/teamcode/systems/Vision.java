package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.FilteredPose3D;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

public class Vision extends StealthSubsystem {

    private final Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
    private static final int  RED_LEFT_PIPELINE = 0, RED_RIGHT_PIPELINE = 1, BLUE_RIGHT_PIPELINE = 2, BLUE_LEFT_PIPELINE = 3;
//    private static final Set<Integer> RED_LEFT_TAG_IDS = Set.of(30, 31, 32, 33);
//    private static final Set<Integer> RED_RIGHT_TAG_IDS = Set.of(34, 35, 36, 37);
//    private static final Set<Integer> BLUE_RIGHT_TAG_IDS = Set.of(38, 39, 40, 41);
//    private static final Set<Integer> BLUE_LEFT_TAG_IDS = Set.of(42, 43, 44, 45);
    private static final double PIVOT_Y = 71.5; // pivot y in pedro coordinates
    private static final double PIVOT_Z = 43.95; // pivot height in inches //TODO: Refine
    private int currentPipeline = 1;
    private final Limelight3A limelight;
    private boolean isRightSideUp = true;
    private final FilteredPose3D botPose;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(20);
        limelight.pipelineSwitch(0);

        if (Alliance.get() == Alliance.BLUE) {
            setPipeline(RED_RIGHT_PIPELINE); // Pipeline selection based on side facing upwards
        } else {
            setPipeline(BLUE_RIGHT_PIPELINE);
        }

        botPose = new FilteredPose3D(0.1); //TODO: tune
    }

    public Command track() {
        return new InstantCommand(
                limelight::start
        );
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        currentPipeline = pipeline;
    }

    public boolean isRightUp() {
        return isRightSideUp;
    }

    public Pose getPose() {
        return botPose.get().asPose();
    }

    private void switchPipeline() { // Switch pipeline when limelight determines its using the wrong hive state
        switch (currentPipeline) {
            case RED_LEFT_PIPELINE: setPipeline(RED_RIGHT_PIPELINE); isRightSideUp = true; break;
            case RED_RIGHT_PIPELINE: setPipeline(RED_LEFT_PIPELINE); isRightSideUp = false; break;
            case BLUE_RIGHT_PIPELINE: setPipeline(BLUE_LEFT_PIPELINE); isRightSideUp = false; break;
            case BLUE_LEFT_PIPELINE: setPipeline(BLUE_RIGHT_PIPELINE); isRightSideUp = true; break;
            default: break;
        }
    }


    // Calculations can be visualized at https://www.desmos.com/calculator/74lsdinv9r
    public Pose3D getPose(LLResult snapshot) {
        Position LLBotpose = snapshot.getBotpose().getPosition().toUnit(DistanceUnit.INCH); // Botpose reading in limelight coordinates
        double x = LLBotpose.x;
        double y = LLBotpose.y;
        double z = LLBotpose.z;

        Pose3D botPose = new Pose3D(y + 72, 72 - x, z); // Botpose in pedro coordinates - from https://pedropathing.com/docs/pathing/reference/coordinates

        double pivotDistance2 = Math.pow(botPose.y() - PIVOT_Y,2) + Math.pow(botPose.z() - PIVOT_Z,2); // Squared distance between the bot and the pivot axis

        // Calculation from desmos, with an added  + PIVOT_Y term to account for an off-center axis. Note that z is assumed to be 0 by nature of the calculations.
        return new Pose3D(botPose.x(), Math.sqrt(pivotDistance2 - PIVOT_Y * PIVOT_Y) + PIVOT_Y, 0);
    }

    @Override
    public void periodic() {
        LLResult snapshot = limelight.getLatestResult();
        dashboard.addData("isnull", snapshot == null);

        if(snapshot != null && snapshot.isValid()) {
            double pitchMagnitude = Math.abs(snapshot.getBotpose().getOrientation().getPitch(AngleUnit.DEGREES)); // Used to determine hive state
            if(pitchMagnitude > 45) { // Bot thinks it is facing significantly upwards or downwards - signifying incorrect pipeline
                switchPipeline();
                return; // Bot needs to wait until subsequent loop to use snapshot
            } else if(pitchMagnitude > 10) { // Bot thinks hive is in a transitional state and does not collect position
                return; // Delay until adequate reading is acquired
            }

            botPose.update(getPose(snapshot));
//            if(botPose.numReadings() > 10) { // Uncomment when code begins working
//                limelight.stop();
//                botPose.clear();
//            }

            dashboard.addData("ll botpose reading","(%.2f, %.2f, %.2f)", botPose.get().x(), botPose.get().y(), botPose.get().z());

            dashboard.addData("rightsideup", isRightSideUp);
        }
    }
}
