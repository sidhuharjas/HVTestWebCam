# HVTestWebCam — Control Hub Ball CV (Beta Tester Setup)

**Goal:** Get ball detection + chase working on the Control Hub in ~3 minutes with zero code edits.

This is the OpenCV (HSV blob) version from Hive-Vision `cv/` track, packaged to drop straight into the FTC SDK's `TeamCode`. No Limelight, no coprocessor, no model. It runs entirely on the Control Hub.

## What you get

| File | What it does |
|---|---|
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BallBlobPipeline.java` | Shared VisionProcessor base (best candidate selection, shape/size gating). |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BallDetectorPipeline.java` | HSV thresholding per color (yellow/red/blue), scaled for 1280×720. |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BallGetSdkOpMode.java` | **Recommended.** Search → Approach → Grab → Done. Uses multi-frame confirmation so it won't chase false blobs. Drives mecanum wheels + optional intake. |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RawBallGet.java` | 1-file fallback. Minimal state machine (SEARCH/CHASE/DONE). Easier to read, less filtering. |

## Step 1: Download / open FtcRobotController

1. Download or clone [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController.git) (Android Studio project).
2. Open it in **Android Studio** (Ladybug/Koala or recent). Let it sync Gradle fully.

## Step 2: Drop in these files

1. In Android Studio, open the `TeamCode` module in the Project view (`TeamCode > src > main > java > org.firstinspires.ftc.teamcode`).
2. **Delete** any example OpModes you don't want (optional). Do **not** delete `RobotAutoDriveToAprilTagOmni.java`/others unless you know why.
3. Drag/copy these 4 Java files from this repo's `TeamCode/...` folder **directly** into that `teamcode` package in Android Studio.
4. Android Studio will auto-add them. Sync if prompted.

**That's it for code.** No Gradle changes needed.

## Step 3: Robot Configuration (RC app)

On your Driver Station + Control Hub:

1. Open the **FTC Driver Station** app, connect to Control Hub.
2. Go to **Configure Robot** → Edit your config.
3. Add/verify these **names match exactly**:

| Device Type | Name | Notes |
|---|---|---|
| Camera | `Webcam 1` | USB webcam plugged into Control Hub. Use 1280×720 if possible. |
| DcMotor | `fl`, `fr`, `bl`, `br` | Drive wheels (mecanum). |
| DcMotor | `intake` | **Optional.** If missing, the OpModes will still drive to the ball (no intake spin). |

**Motor directions:** `+` power must make the robot go **forward** on all 4 wheels. If any side runs backwards, flip directions in `BallGetSdkOpMode` (`LEFT_DIR` / `RIGHT_DIR`).

## Step 4: Which OpMode to run

On the Driver Station → **TeleOp** dropdown:

- **`Ball Get (OpenCV, no Pedro)`** — **Recommended for beta testing.** Has multi-frame confirmation (`CONFIRM_FRAMES`) so it rejects flickering blobs. Much less jittery.
- **`Raw Ball Get (1 file)`** — Barebones, chases the first blob it sees. Good for sanity-checking HSV/tuning only.

Select the recommended one first.

## Step 5: Tuning (only if it chases the wrong thing)

The defaults are tuned for FTC game balls under typical field lighting (scaled for 1280×720). If it's grabbing tape/panels:

1. **Tweak color first.** The easiest way: run `hsv_tuner.py` from [Hive-Vision/cv/tools](https://github.com/hivevisionftc/Hive-Vision/tree/main/cv/tools) on a recorded frame from your webcam. Copy `H/S/V` min/max back into `BallDetectorPipeline.java` (YELLOW/RED/BLUE blocks).
2. **Distance scaling.** Park robot exactly **24"** from a ball. Look at telemetry `ball dist=... in`. If it's off, adjust `DIST_REF_AREA` in `BallGetSdkOpMode` (larger area → thinks it's closer). Formula: `newArea ≈ oldArea * (shownDist/realDist)^2`.
3. **Confirmation.** **Do not** drop `CONFIRM_FRAMES` below 3–4 to "feel faster". Fix lighting/color instead — that's the intended behavior (candidate detector).

## Tips that save time

- **LiveView:** VisionPortal has LiveView enabled — you can see thresholded blobs on the Driver Station/RC camera stream while tuning.
- **Red wrap-around:** Red is handled as two HSV ranges (standard OpenCV hue wrap). Defaults cover most lighting.
- **Grab distance:** `GRAB_DIST_IN=16"` keeps the ball far enough that area gates don't reject very close blobs at 720p.
- **Lost timeout:** If the ball leaves frame, it backs to `SEARCH` after `LOST_MS=400ms` and spins.

## Expected behavior

1. TeleOp starts in `SEARCH` → robot slowly spins in place.
2. Aim webcam at a ball → after ~4 consistent frames it locks on (`APPROACH`).
3. Steers to center, drives forward until ~16" away → enters `GRAB`, runs intake, creeps forward until ball disappears → `DONE` (stops).

If it never locks on: lighting is the #1 culprit. Tune HSV ranges before changing gains.

**Questions/bugs?** File an issue on [HVTestWebCam](https://github.com/sidhuharjas/HVTestWebCam). This pack tracks the `cv/` Control Hub track from Hive-Vision.
