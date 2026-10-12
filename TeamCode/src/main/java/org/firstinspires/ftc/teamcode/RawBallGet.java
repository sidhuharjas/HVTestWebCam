/*
 * RawBallGet.java - the whole "go fetch the ball" demo in ONE file.
 *
 * Dependencies: the FTC SDK, nothing else. No Pedro, no other project
 * files, no external libraries - VisionPortal + OpenCV ship with the SDK.
 *
 * Drop this single file into TeamCode, config the hardware names below,
 * run as TeleOp:
 *
 *   SEARCH  spin until a blob shows up
 *   CHASE   steer on blob x, drive on distance (area -> inches)
 *   DONE    inside GRAB_DIST_IN - stop everything
 *
 * This is the raw/minimal version: it trusts every detection. The
 * hivevisionftc cv/ README shows why you'd normally want multi-frame
 * confirmation (precision 26-66% by color) - see BallGetSdkOpMode.
 */
package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@TeleOp(name = "Raw Ball Get (1 file)")
public class RawBallGet extends OpMode {

    /* ---- TUNE ME: hardware ---- */
    private static final String CAM = "Webcam 1";
    private static final String INTAKE = "intake"; // optional - omit the motor and it still drives

    /* ---- TUNE ME: ball color (HSV, OpenCV hue 0-180) ---- */
    private static final Scalar COLOR_LO = new Scalar(102, 80, 45); // blue; yellow = (9,90,45), red = (0,81,45)+(170,81,45)
    private static final Scalar COLOR_HI = new Scalar(128, 255, 247);

    /* ---- distance from blob area: dist = REF_IN * sqrt(REF_AREA / area) ---- */
    private static final double DIST_REF_IN = 24.0;
    private static final double DIST_REF_AREA = 1865.0; // area at DIST_REF_IN, px^2 at 1280x720
    private static final double MIN_AREA_PX = 100.0;    // ignore specks
    private static final double GRAB_DIST_IN = 16.0;    // stop this far short of the ball

    /* ---- driving ---- */
    private static final double SEARCH_SPIN = 0.3;
    private static final double TURN_KP = 1.6;
    private static final double DRIVE_KP = 0.05;
    private static final double INTAKE_POWER = 0.8;
    private static final long LOST_MS = 400; // no blob this long -> back to SEARCH

    private enum State {
        SEARCH,
        CHASE,
        DONE
    }

    private final ElapsedTime clock = new ElapsedTime();
    private State state = State.SEARCH;
    private long lastSeenMs = 0;

    private DcMotor fl, fr, bl, br, intake;
    private BlobDetector detector;
    private VisionPortal portal;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        intake = hardwareMap.tryGet(DcMotor.class, INTAKE);
        if (intake != null) intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        detector = new BlobDetector();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAM))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(detector)
                .enableLiveView(true)
                .build();
    }

    @Override
    public void loop() {
        boolean found = detector.found();
        if (found) lastSeenMs = (long) clock.milliseconds();

        switch (state) {
            case SEARCH:
                drive(0, SEARCH_SPIN);
                if (found) state = State.CHASE;
                break;

            case CHASE:
                if (!found) {
                    if (clock.milliseconds() - lastSeenMs > LOST_MS) {
                        state = State.SEARCH; // ball walked off - sweep again
                    } else {
                        drive(0, SEARCH_SPIN); // just missed a frame, keep turning
                    }
                    break;
                }
                double turn = TURN_KP * (0.5 - detector.cxNorm); // cxNorm 0.5 = dead ahead
                double dist = DIST_REF_IN * Math.sqrt(DIST_REF_AREA / detector.area);
                double fwd = Range.clip(DRIVE_KP * (dist - GRAB_DIST_IN), 0, 1);
                drive(turn, fwd);
                if (intake != null) intake.setPower(INTAKE_POWER);
                if (dist <= GRAB_DIST_IN) state = State.DONE;
                break;

            case DONE:
            default:
                stopAll();
                break;
        }

        telemetry.addData("state", state);
        if (found) {
            telemetry.addData(
                    "ball",
                    "x=%.3f  dist=%.1f in",
                    detector.cxNorm,
                    DIST_REF_IN * Math.sqrt(DIST_REF_AREA / detector.area));
        } else {
            telemetry.addData("ball", "none");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAll();
        if (portal != null) portal.close();
    }

    /** Mecanum mix: turn = CCW+, fwd = robot-forward. */
    private void drive(double fwd, double turn) {
        fl.setPower(fwd - turn);
        bl.setPower(fwd - turn);
        fr.setPower(fwd + turn);
        br.setPower(fwd + turn);
    }

    private void stopAll() {
        drive(0, 0);
        if (intake != null) intake.setPower(0);
    }

    /** Largest color blob in the frame: cxNorm in [0,1], area in px^2. */
    static class BlobDetector implements VisionProcessor {
        volatile double cxNorm = -1;
        volatile double area = 0;

        boolean found() {
            return cxNorm >= 0;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {}

        @Override
        public Object processFrame(Mat rgba, long captureTimeNs) {
            Mat hsv = new Mat();
            Mat mask = new Mat();
            Mat hierarchy = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();
            try {
                Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);
                Core.inRange(hsv, COLOR_LO, COLOR_HI, mask);
                Imgproc.findContours(
                        mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                Rect best = null;
                for (MatOfPoint c : contours) {
                    Rect r = Imgproc.boundingRect(c);
                    if (best == null || r.area() > best.area()) best = r;
                    c.release();
                }
                if (best != null && best.area() >= MIN_AREA_PX) {
                    cxNorm = (best.x + best.width / 2.0) / rgba.cols();
                    area = best.area();
                } else {
                    cxNorm = -1;
                    area = 0;
                }
            } finally {
                hsv.release();
                mask.release();
                hierarchy.release();
            }
            return null;
        }

        @Override
        public void onDrawFrame(
                Canvas canvas, int width, int height, float scaleX, float scaleY, Object object) {
            // raw version draws nothing; BallBlobPipeline paints boxes + labels
        }
    }
}
