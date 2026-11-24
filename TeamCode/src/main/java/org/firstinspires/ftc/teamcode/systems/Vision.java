package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Vision extends StealthSubsystem {
    private static final Set<Integer> RED_LEFT_TAG_IDS = Set.of(30, 31, 32, 33); // From https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual#page=77
    private static final Set<Integer> RED_RIGHT_TAG_IDS = Set.of(34, 35, 36, 37);
    private static final Set<Integer> BLUE_RIGHT_TAG_IDS = Set.of(38, 39, 40, 41);
    private static final Set<Integer> BLUE_LEFT_TAG_IDS = Set.of(42, 43, 44, 45);
    private static final double UP_THRESHOLD_PIXELS = 240; // Represents the threshold of TargetPixelY to be considered tipped up
    private static final double CONFIDENCE_THRESHOLD = 0.5; // Confidence level required to turn off camera
    private final Limelight3A limelight;
    private boolean isRightSideUp = true;
    private boolean isConfident = false;
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

    public Command updateLimelight() {
        return new InstantCommand(
                () -> {
                    limelight.start();
                    isConfident = false;
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

        if(snapshot != null && snapshot.isValid()) {
            List<FiducialResult> tags = snapshot.getFiducialResults();
            int rightUpReadings = 0; // Tracks the number of readings that show right side as up

            for(FiducialResult tag : tags) {
                if(RIGHT_TAGS.contains(tag.getFiducialId()) && tag.getTargetYPixels() > UP_THRESHOLD_PIXELS || // Right up contributes to upward confidence
                        LEFT_TAGS.contains(tag.getFiducialId()) && tag.getTargetYPixels() < UP_THRESHOLD_PIXELS) { // Left down contributes to upward confidence
                    rightUpReadings++;
                } else {
                    rightUpReadings--;
                }
            }

            double rightUpConfidence = (double) rightUpReadings / tags.size();

            if(rightUpConfidence > CONFIDENCE_THRESHOLD) isRightSideUp = true;
            if(rightUpConfidence < -CONFIDENCE_THRESHOLD) isRightSideUp = false;

            if(Math.abs(rightUpConfidence) > CONFIDENCE_THRESHOLD) { // Limelight is confident, the limelight stops
                isConfident = true;
                limelight.stop();
            }
        }
    }

}
