/*
 * BallGetSdkOpMode - "go fetch a ball" with NOTHING but the FTC SDK.
 *
 * Detection comes from the hivevisionftc/Hive-Vision cv/ track
 * (BallDetectorPipeline: HSV color thresholds on the Control Hub - no
 * coprocessor, no model, no Limelight). This OpMode closes the loop
 * camera -> mecanum powers:
 *
 *   SEARCH   spin until the same blob has been confirmed for CONFIRM_FRAMES
 *   APPROACH steer on cxNorm, drive on distance (blob area), grab when close
 *   GRAB     stop, run the intake until the ball disappears from the frame
 *   DONE     everything off
 *
 * Distance comes from blob area: twice as far = quarter of the pixels, so
 *   dist = DIST_REF_IN * sqrt(DIST_REF_AREA / area).
 * Calibrate DIST_REF_AREA by reading telemetry at a tape-measure distance.
 *
 * The cv/ detector is a CANDIDATE detector (measured precision: yellow 40%,
 * red 26%, blue 66%) - that is exactly why nothing moves until the same
 * blob has held position for CONFIRM_FRAMES consecutive frames. See the
 * upstream cv/README.md.
 */
package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Ball Get (OpenCV, no Pedro)")
public class BallGetSdkOpMode extends OpMode {

    /* ---- TUNE ME: hardware names ---- */
    private static final String CAM_NAME = "Webcam 1";
    private static final String FL = "fl";
    private static final String FR = "fr";
    private static final String BL = "bl";
    private static final String BR = "br";
    private static final String INTAKE = "intake"; // optional: leave the motor off to just drive

    /* TUNE ME: flip the side(s) whose motors run backwards; +power must mean robot-forward. */
    private static final DcMotorSimple.Direction LEFT_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction RIGHT_DIR = DcMotorSimple.Direction.REVERSE;

    /* ---- TUNE ME: which ball to fetch ---- */
    private static final BallBlobPipeline.BallColor TARGET = BallBlobPipeline.BallColor.BLUE;

    /* ---- camera / distance ---- */
    private static final double HFOV_DEG = 65.0;     // webcam horizontal FOV, degrees
    private static final double DIST_REF_IN = 24.0;  // DIST_REF_AREA was measured at this range...
    private static final double DIST_REF_AREA = 1865.0; // ...blob area there, px^2 at 1280x720

    /* ---- confirmation (cv/README: candidate detector - require N frames) ---- */
    private static final int CONFIRM_FRAMES = 4;
    private static final double CONFIRM_DX = 0.025;        // cxNorm jitter allowed between frames
    private static final double CONFIRM_AREA_RATIO = 0.35; // area jitter allowed between frames
    private static final long LOST_MS = 400;               // no blob this long -> back to SEARCH

    /* ---- driving ---- */
    private static final double SEARCH_SPIN = 0.3; // in-place sweep while looking
    private static final double TURN_KP = 1.2;     // turn power per unit of cx error
    private static final double MAX_TURN = 0.5;
    private static final double AIM_TOL = 0.12;    // only drive forward while this centred (0..0.5)
    private static final double DRIVE_KP = 0.04;   // forward power per inch still to close
    private static final double MIN_FWD = 0.15;
    private static final double MAX_FWD = 0.6;

    /* ---- grab ---- */
    private static final double GRAB_DIST_IN = 16.0; // keep >= ~14" at 720p: MAX_AREA_PX rejects closer blobs
    private static final double INTAKE_POWER = 0.8;
    private static final double GRAB_CREEP = 0.15;   // forward power while the intake runs
    private static final long GRAB_TIMEOUT_MS = 2000;
    private static final int GONE_FRAMES = 3;        // consecutive empty frames = ball swallowed

    private enum State {
        SEARCH,
        APPROACH,
        GRAB,
        DONE
    }

    private DcMotor fl, fr, bl, br, intake;
    private BallDetectorPipeline pipeline;
    private VisionPortal portal;
    private State state = State.SEARCH;
    private final ElapsedTime clock = new ElapsedTime();

    private BallBlobPipeline.BallBlob blob;
    private int confirmFrames = 0;
    private double lastCx = -1, lastArea = -1;
    private long lastSeenMs = 0;
    private int goneFrames = 0;
    private long grabStartMs = 0;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, FL);
        fr = hardwareMap.get(DcMotor.class, FR);
        bl = hardwareMap.get(DcMotor.class, BL);
        br = hardwareMap.get(DcMotor.class, BR);
        for (DcMotor m : new DcMotor[] {fl, fr, bl, br}) {
            m.setDirection(m == fl || m == bl ? LEFT_DIR : RIGHT_DIR);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake = hardwareMap.tryGet(DcMotor.class, INTAKE);
        if (intake != null) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        pipeline = new BallDetectorPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAM_NAME))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(pipeline)
                .enableLiveView(true)
                .build();
    }

    @Override
    public void loop() {
        readBlob();

        switch (state) {
            case SEARCH:
                drive(0, SEARCH_SPIN);
                if (confirmed()) {
                    state = State.APPROACH;
                }
                break;

            case APPROACH:
                if (lost()) {
                    state = State.SEARCH;
                    break;
                }
                if (blob == null) {
                    drive(0, 0); // within the lost-grace window: hold still
                    break;
                }
                double d = dist();
                if (d <= GRAB_DIST_IN) {
                    enterGrab();
                    break;
                }
                double err = blob.cxNorm - 0.5; // + = ball right of centre
                double turn = Range.clip(-err * 2.0 * TURN_KP, -MAX_TURN, MAX_TURN);
                double fwd = 0;
                if (Math.abs(err) < AIM_TOL) {
                    fwd = Range.clip((d - GRAB_DIST_IN) * DRIVE_KP, MIN_FWD, MAX_FWD);
                }
                drive(fwd, turn);
                break;

            case GRAB:
                goneFrames = blob == null ? goneFrames + 1 : 0;
                boolean swallowed = goneFrames >= GONE_FRAMES;
                boolean timedOut = clock.milliseconds() - grabStartMs > GRAB_TIMEOUT_MS;
                if (swallowed || timedOut) {
                    if (intake != null) intake.setPower(0);
                    drive(0, 0);
                    state = State.DONE;
                    break;
                }
                if (intake != null) intake.setPower(INTAKE_POWER);
                drive(GRAB_CREEP, 0);
                break;

            case DONE:
            default:
                drive(0, 0);
                if (intake != null) intake.setPower(0);
                break;
        }

        telemetry.addData("state", "%s   confirm %d/%d%s", state, confirmFrames, CONFIRM_FRAMES, lost() ? "  LOST" : "");
        if (blob != null) {
            telemetry.addData("ball cx", "%.3f", blob.cxNorm);
            telemetry.addData("ball dist", "%.1f in", dist());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        drive(0, 0);
        if (intake != null) intake.setPower(0);
        if (portal != null) portal.close();
    }

    /* ---------------- detection bookkeeping ---------------- */

    private void readBlob() {
        blob = pipeline.bestOf(TARGET);
        if (blob == null) return;

        long now = (long) clock.milliseconds();
        boolean same = lastCx >= 0
                && Math.abs(blob.cxNorm - lastCx) <= CONFIRM_DX
                && Math.abs(blob.area / lastArea - 1.0) <= CONFIRM_AREA_RATIO;
        confirmFrames = same ? Math.min(confirmFrames + 1, CONFIRM_FRAMES) : 1;
        lastCx = blob.cxNorm;
        lastArea = blob.area;
        lastSeenMs = now;
    }

    private boolean confirmed() {
        return blob != null && confirmFrames >= CONFIRM_FRAMES;
    }

    private boolean lost() {
        return clock.milliseconds() - lastSeenMs > LOST_MS;
    }

    private double dist() {
        return DIST_REF_IN * Math.sqrt(DIST_REF_AREA / Math.max(blob.area, 1.0));
    }

    /* ---------------- actuators ---------------- */

    private void enterGrab() {
        drive(0, 0);
        if (intake != null) intake.setPower(INTAKE_POWER);
        goneFrames = 0;
        grabStartMs = (long) clock.milliseconds();
        state = State.GRAB;
    }

    /** Mecanum mix, CCW+ turn: left column = fwd - turn, right column = fwd + turn. */
    private void drive(double fwd, double turn) {
        if (fl == null) return; // init failed: nothing to drive
        double l = fwd - turn;
        double r = fwd + turn;
        fl.setPower(l);
        bl.setPower(l);
        fr.setPower(r);
        br.setPower(r);
    }
}
