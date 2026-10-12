# HVTestWebCam — Control Hub OpenCV Ball Chase (Beta Test)

Part of [Hive Vision](https://github.com/hivevisionftc/Hive-Vision) (model, training and calibration tooling).

Quick, drop-in OpenCV ball detection for the Control Hub. Runs entirely on the robot with a USB webcam (VisionPortal + OpenCV) — **no Limelight, no coprocessor, and no external model required**. This pack is built to work with the official FTC SDK and is designed to be as easy as possible for beta testers.

## What does Gradle do?

Gradle is Android Studio's build tool. For FTC projects it:

- **Downloads dependencies** (FTC SDK, Pedro Pathing, etc.) so you don't have to copy `.jar` files by hand.
- **Compiles your code** (`TeamCode`) together with the FTC SDK.
- **Builds and deploys** the app to the Control Hub/Robot Controller when you hit "Run" in Android Studio.

You don't need to touch Gradle for this pack — just drop the Java files into `TeamCode`.

## Installation

Pick one of the two ways to install this for testing.

| Method | Difficulty | When to use |
|---|---|---|
| **Drop-in (easiest)** | 30 seconds | Beta testing this repo as-is. No Gradle edits required. |
| **Install from source (local)** | 1–2 minutes | If you want to track changes or test multiple local packs in the same FtcRobotController checkout. |

### Option 1: Drop-in (Recommended, easiest for beta testers)

1. Clone or download [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController.git).
2. Open **FtcRobotController** in Android Studio. Let Gradle sync fully.
3. In Android Studio, navigate to `TeamCode > src > main > java > org.firstinspires.ftc.teamcode`.
4. Copy **all 4 Java files** from this repo's `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` into that folder:

   - `BallBlobPipeline.java`
   - `BallDetectorPipeline.java`
   - `BallGetSdkOpMode.java`
   - `RawBallGet.java`

5. Build and deploy to the Control Hub. You're done.

### Option 2: Beta testing from source (local Maven)

If you want to test this as a local library instead of copying files:

1. Clone this repo:

   ```bash
   git clone https://github.com/sidhuharjas/HVTestWebCam.git
   cd HVTestWebCam
   ```

2. Publish to your local Maven repo (`~/.m2/repository`):

   ```bash
   ./gradlew publishToMavenLocal
   ```

   > Needs JDK 17+ and Android SDK set (`ANDROID_HOME` or `local.properties` with `sdk.dir`).

3. Open your **FtcRobotController** project. In `build.dependencies.gradle`, add `mavenLocal()` **before** `mavenCentral()`:

   ```gradle
   repositories {
       mavenLocal()
       mavenCentral()
       google()
   }
   ```

4. In `TeamCode/build.gradle`, add the dependency (example):

   ```gradle
   dependencies {
       implementation project(':FtcRobotController')   // already there
       implementation 'org.hivevisionftc:hvtestwebcam:0.1.0-SNAPSHOT'
   }
   ```

   > If the version differs, check `gradle.properties` in this repo.

5. Copy the example OpModes from `docs/examples/` (if present) into `TeamCode`, or just use the ones published. Sync Gradle and deploy.

> **Note:** Most beta testers should use **Option 1 (Drop-in)**. It's the easiest and requires zero Gradle edits.

## Hardware Configuration (Control Hub)

In the **FTC Driver Station > Configure Robot** menu, create/verify these device names **exactly** as listed:

| Device Type | Name | Notes |
|---|---|---|
| Camera | `Webcam 1` | USB webcam plugged into the Control Hub. Run at **1280×720** for best results. |
| DcMotor | `fl`, `fr`, `bl`, `br` | Mecanum drive wheels. |
| DcMotor | `intake` | **Optional.** If not present, the OpModes will still drive to the ball (intake won't spin). |

**Motor directions:** `+` power must make the robot move **forward** on all four wheels. If any side runs backwards, flip `LEFT_DIR` / `RIGHT_DIR` in `BallGetSdkOpMode`.

## Which OpMode to Run

In TeleOp, select one of these:

| OpMode Name | Use For | Notes |
|---|---|---|
| **`Ball Get (OpenCV, no Pedro)`** | **Recommended (beta testing)** | Multi-frame confirmation (`CONFIRM_FRAMES`) prevents chasing flickering blobs. Smoothest and least jittery. |
| **`Raw Ball Get (1 file)`** | Sanity check / tuning | Barebones (SEARCH/CHASE/DONE). Chases the first valid blob it sees. Great for quick HSV verification. |

Run **`Ball Get (OpenCV, no Pedro)`** first.

## How It Works

Both OpModes follow the same core flow:

```text
SEARCH    Spin in place until a stable blob is detected
APPROACH  Steer on cxNorm, drive on distance (blob area → inches)
GRAB      Stop, run intake, creep forward until ball disappears from frame (or timeout)
DONE      All motors off
```

**Distance estimation:** `dist = DIST_REF_IN * sqrt(DIST_REF_AREA / area)`. Twice as far = ¼ the pixels. Calibrate once with a tape measure at 24".

**Confirmation (why it waits):** OpenCV HSV blob detection is a *candidate* detector (not ground truth). Requiring the same blob for **4 consecutive frames** (`CONFIRM_FRAMES=4`) prevents false positives from tape/panels. **Do not** lower this to feel snappier — fix lighting/HSV ranges instead.

## Tuning (If It Chases the Wrong Thing)

Defaults are tuned for FTC game balls under typical field lighting at 1280×720. If it's picking up tape/robot panels:

1. **Tune HSV first (biggest win).** Use [`cv/tools/hsv_tuner.py`](https://github.com/hivevisionftc/Hive-Vision/tree/main/cv/tools) from Hive-Vision on a frame from your webcam. Copy the H/S/V min/max back into `BallDetectorPipeline.java` (YELLOW/RED/BLUE blocks).
2. **Calibrate distance.** Park the robot exactly **24"** from a ball. Look at telemetry: `ball dist=... in`. If off, adjust `DIST_REF_AREA` in `BallGetSdkOpMode` proportionally: `newArea ≈ oldArea * (shownDist/realDist)^2`.
3. **Keep confirmation high.** Leave `CONFIRM_FRAMES` at 3–4. Lighting fixes almost always beat lowering filters.

## Tips

- **LiveView:** VisionPortal has LiveView enabled — you can see thresholded blobs on the Driver Station while tuning.
- **Red hue wrap:** Red is automatically handled with two HSV ranges (OpenCV hue wraps at 0/180).
- **Grab distance:** `GRAB_DIST_IN=16"` avoids rejecting blobs that get very large up close at 720p.
- **Lost timeout:** If the ball leaves the frame, it returns to `SEARCH` after `LOST_MS=400ms`.

## Expected Behavior

1. TeleOp starts → `SEARCH` (slow spin).
2. Aim at a ball → after ~4 consistent frames it locks (`APPROACH`).
3. Centers + drives forward to ~16" → `GRAB`, runs intake, creeps until ball disappears → `DONE` (stops).

If it never locks on, **lighting/color ranges** are almost always the culprit.

## Building & Testing (Developers)

From this repo root:

```bash
./gradlew test               # Desktop unit tests (no robot/camera)
./gradlew assembleRelease    # Build AAR + sources/javadoc
./gradlew spotlessCheck      # Formatting check (spotlessApply to fix)
```

Requires JDK 17+ and Android SDK (`ANDROID_HOME` or `local.properties` with `sdk.dir`).

## License

MIT — see [LICENSE](LICENSE) if present.
