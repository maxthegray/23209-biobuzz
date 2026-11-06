package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

// shows which hive tags the limelight sees. press A, tip the hive by hand, and see how far the tags moved
@TeleOp(name = "Limelight Hive Tag Check", group = "test")
public class LimelightHiveCheck extends LinearOpMode {

    private static final int tagPipeline = 0;
    private static final String[] cellNames = {"red rear", "red audience", "blue audience", "blue rear"};

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(tagPipeline);
        limelight.start();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        double[] savedTy = new double[4];
        boolean[] savedSeen = new boolean[4];
        boolean haveSnapshot = false;
        boolean aWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            follower.manual(-gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_x * 0.5, -gamepad1.right_stick_x * 0.4);
            follower.update();

            double[] sumTx = new double[4], sumTy = new double[4], sumArea = new double[4];
            int[] count = new int[4];
            StringBuilder idsSeen = new StringBuilder();

            LLResult result = limelight.getLatestResult();
            // right after a pipeline switch you can still get frames from the old one
            if (result != null && result.isValid() && result.getPipelineIndex() == tagPipeline) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : tags) {
                    int id = tag.getFiducialId();
                    idsSeen.append(id).append(' ');
                    int cell = cellForTag(id);
                    if (cell < 0) continue;
                    sumTx[cell] += tag.getTargetXDegrees();
                    sumTy[cell] += tag.getTargetYDegrees();
                    sumArea[cell] += tag.getTargetArea();
                    count[cell]++;
                }
            }

            if (gamepad1.a && !aWasPressed) {
                for (int c = 0; c < 4; c++) {
                    savedSeen[c] = count[c] > 0;
                    savedTy[c] = count[c] > 0 ? sumTy[c] / count[c] : 0;
                }
                haveSnapshot = true;
            }
            aWasPressed = gamepad1.a;

            LLStatus llStatus = limelight.getStatus();
            telemetry.addData("Limelight", limelight.isConnected() ? "connected" : "NOT CONNECTED");
            telemetry.addData("Pipeline", "asked for %d, running %d (%s), %.0f fps",
                    tagPipeline, llStatus.getPipelineIndex(), llStatus.getPipelineType(), llStatus.getFps());
            telemetry.addData("Tags seen", idsSeen.length() > 0 ? idsSeen.toString() : "none");
            telemetry.addLine();
            telemetry.addLine("cell           tags   tx       ty       area");
            for (int c = 0; c < 4; c++) {
                if (count[c] == 0) {
                    telemetry.addLine(String.format("%-14s  -", cellNames[c]));
                    continue;
                }
                double ty = sumTy[c] / count[c];
                String moved = haveSnapshot && savedSeen[c] ? String.format("  (moved %+.1f)", ty - savedTy[c]) : "";
                telemetry.addLine(String.format("%-14s %d/4   %+6.1f   %+6.1f   %5.2f%s",
                        cellNames[c], count[c], sumTx[c] / count[c], ty, sumArea[c] / count[c], moved));
            }
            telemetry.addLine();
            telemetry.addLine(haveSnapshot ? "A = new snapshot" : "A = save snapshot, then tip the hive");
            telemetry.update();
        }

        limelight.stop();
        follower.stop();
        follower.update();
    }

    // tag ids per cell: 30-33 red rear, 34-37 red audience, 38-41 blue audience, 42-45 blue rear
    private static int cellForTag(int id) {
        if (id < 30 || id > 45) return -1;
        return (id - 30) / 4;
    }
}
