package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Vision extends StealthSubsystem {

    private final Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
    private static final Set<Integer> RED_LEFT_TAG_IDS = Set.of(30, 31, 32, 33); // From https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual#page=77
    private static final Set<Integer> RED_RIGHT_TAG_IDS = Set.of(34, 35, 36, 37);
    private static final Set<Integer> BLUE_RIGHT_TAG_IDS = Set.of(38, 39, 40, 41);
    private static final Set<Integer> BLUE_LEFT_TAG_IDS = Set.of(42, 43, 44, 45);
    private final Limelight3A limelight;
    private boolean tracking_left = false;
    private boolean tracking_right = false;
    private boolean isRightSideUp = true;
    private final Set<Integer> LEFT_TAGS;
    private final Set<Integer> RIGHT_TAGS;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(20);
        limelight.pipelineSwitch(0);

        if (Alliance.get() == Alliance.BLUE) {
            LEFT_TAGS = BLUE_LEFT_TAG_IDS;
            RIGHT_TAGS = BLUE_RIGHT_TAG_IDS;
        } else {
            LEFT_TAGS = RED_LEFT_TAG_IDS;
            RIGHT_TAGS = RED_RIGHT_TAG_IDS;
        }
    }

    public Command trackLeftSide() {
        return new InstantCommand(
                () -> {
                    limelight.start();
                    tracking_right = false;
                    tracking_left = true;
                }
        );
    }
    public Command trackRightSide() {
        return new InstantCommand(
                () -> {
                    limelight.start();
                    tracking_left = false;
                    tracking_right = true;
                }
        );
    }

    public boolean isLeftUp() {
        return !isRightSideUp;
    }
    public boolean isRightUp() {
        return isRightSideUp;
    }

    @Override
    public void periodic() {
        LLResult snapshot = limelight.getLatestResult();
        dashboard.addData("isnull", snapshot == null);

        if(snapshot != null && snapshot.isValid()) {
            List<FiducialResult> tags = snapshot.getFiducialResults();

            dashboard.addData("First tag id", tags.get(0).getFiducialId());
            dashboard.addData("First tag y", tags.get(0).getTargetYPixels());

            for(FiducialResult tag : tags) {
                if (tracking_right && RIGHT_TAGS.contains(tag.getFiducialId())) {
                    isRightSideUp = true;
                }
                if (tracking_left && LEFT_TAGS.contains(tag.getFiducialId())) {
                    isRightSideUp = false;
                }
            }
            dashboard.addData("rightsideup", isRightSideUp);
        }
    }

}
