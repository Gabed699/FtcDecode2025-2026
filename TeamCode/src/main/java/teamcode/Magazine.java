/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;


/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Magazine", group="Robot")
//@Disabled
public class Magazine extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotorEx magazine = null;

    private PIDController controller;

    public static final double p = 0.2;
    public static final double i = 0;
    public static final double d = 0;
    public static final double f = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private PredominantColorProcessor colorSensor;

    VisionPortal.Builder myVisionPortalBuilder;
    VisionPortal.Builder secondPortalBuilder;
    boolean USE_WEBCAM_1;
    int Portal_1_View_ID;
    boolean USE_WEBCAM_2;
    int Portal_2_View_ID;
    AprilTagProcessor myAprilTagProcessor_1;
    //AprilTagProcessor myAprilTagProcessor_2;
    VisionPortal myVisionPortal_1;
    VisionPortal myVisionPortal_2;


    private void initMultiPortals() {
        List myPortalsList;

        myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, (int) 0, false)).intValue();
        Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, (int) 1, false)).intValue();
        telemetry.addData("Portal 1 View ID (index 0 of myPortalsList)", Portal_1_View_ID);
        telemetry.addData("Portal 2 View ID (index 1 of myPortalsList)", Portal_2_View_ID);
        telemetry.addLine("");
        //telemetry.addLine("Press Y to continue");
        telemetry.update();
        //while (!gamepad1.y && opModeInInit()) {
            // Loop until gamepad Y button is pressed.
        //}
    }

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final float COUNTS_PER_FULL_REV = 96.245f;    // This should be 47.1 at some point but it's fine for now
    float floatTargetPosition = 0;
    //static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    //static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //(WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    //static final double     TURN_SPEED              = 0.5;

    String[] colors = new String[3];
    float slotNumber = 0f;
    float topSlotNumber = 1.5f;
    String aprilTagID = String.valueOf('0');
    int currentMagazinePosition = 0;



    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
        magazine = (DcMotorEx) hardwareMap.get(DcMotor.class, "magazine");
        controller = new PIDController(p, i, d);  // ← ADD THIS LINE HERE

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        magazine.setDirection(DcMotor.Direction.FORWARD);

        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ///PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        ///magazine.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", magazine.getCurrentPosition());
        telemetry.update();
        // Wait for the game to start (driver presses START)

        // This OpMode shows AprilTag recognition and pose estimation.
        USE_WEBCAM_1 = true;
        USE_WEBCAM_2 = true;
        initMultiPortals();
        // Initialize AprilTag before waitForStart.

        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        // PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();


        telemetry.addData("Webcam 1 available", hardwareMap.get(WebcamName.class, "Webcam 1") != null);
        telemetry.addData("Webcam 2 available", hardwareMap.get(WebcamName.class, "Webcam 2") != null);
        telemetry.update();

        initAprilTag();


        /*
         * Build a vision portal to run the Color Sensor process.
         *
         *  - Add the colorSensor process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        //VisionPortal portal = new VisionPortal.Builder()
                //.addProcessor(colorSensor);
                //.setCameraResolution(new Size(320, 240))
                //.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                //.build();

        telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        //PredominantColorProcessor.Result result = colorSensor.getAnalysis();

        waitForStart();
        sleep(500);
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        aprilTagID = AprilTag_telemetry_for_Portal_1();
        while (aprilTagID == null && opModeIsActive()) {
            aprilTagID = AprilTag_telemetry_for_Portal_1();
            telemetry.addData("DEBUG", "aprilTagID returned: " + aprilTagID);
            telemetry.update();
            sleep(50);
        }
        /// PRE LOAD SETTINGS
        colors[0] = "ARTIFACT_PURPLE";
        colors[1] = "ARTIFACT_GREEN";
        colors[2] = "ARTIFACT_PURPLE";
        /// THIS BLOCK IS THE OPERTATOR THAT INTAKES 3 BALLS AND CHECKS THE COLOR. NOT NECESSARY FOR FIRST PASS
        /*for (int i = 0; i < 3; i++) {
            sleep(1000);
            fullRotation(1, 0.3f, true);
            while (magazine.isBusy() && opModeIsActive()) {
                //hold loop while function moving
            }
            telemetry.addData("Best Match", result.closestSwatch.toString());
            telemetry.addData("Color Array", colors[0] + ", " + colors[1] + ", " + colors[2]);
            telemetry.update();
        }
        */
        /// END OF BLOCK THAT IS THE OPERTATOR THAT INTAKES BALLS AND CHECKS THE COLOR.

        fullRotation(0.5f, 0.3f, false);
        //launches based on scanned apriltag
        if (aprilTagID.equals("21")) {
            launchMotif(21);
        }
        if (aprilTagID.equals("22")) {
            launchMotif(22);
        }
        if (aprilTagID.equals("23")) {
            launchMotif(23);
        }


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(3000);  // pause to display final telemetry message.



        //Closes visionportals to fix problems related to cameras being "already attached"
        if (myVisionPortal_1 != null) {
            myVisionPortal_1.close();
        }
        if (myVisionPortal_2 != null) {
            myVisionPortal_2.close();
        }

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    /*public void halfRotation() {
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        sleep(1500);
        if (slotNumber % 1.0 == 0) {
            colors[(int) slotNumber] = result.closestSwatch.toString();
        }
        if (slotNumber < 2.5) {
            slotNumber += 0.5;
        } else {
            slotNumber = 0;
        }
        magazine.setTargetPosition(magazine.getCurrentPosition() + COUNTS_PER_FULL_REV*0.5);
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazine.setPower(DRIVE_SPEED);
    } */
    public void fullRotation(float numOfRotations, float magazinePower, boolean assignNewColor) {
        if (assignNewColor) {
            sleep(1000);
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();
            sleep(1000);

        if (slotNumber % 1.0 == 0) {
            colors[(int) slotNumber] = result.closestSwatch.toString();
            if (!colors[(int) slotNumber].equals("ARTIFACT_PURPLE") && !colors[(int) slotNumber].equals("ARTIFACT_GREEN")) {
                colors[(int) slotNumber] = "ARTIFACT_PURPLE";
            }
        }
    }
        slotNumber += (1 * numOfRotations);
        if (slotNumber >= 3) {
            slotNumber = slotNumber - 3;
        }

        topSlotNumber += (1 * numOfRotations);
        if (topSlotNumber >= 3) {
            topSlotNumber = topSlotNumber - 3;
        }

        //Make motor move with custom PID controller -- This was written by Claude because I don't understand PID controllers

        double power = 0;
        floatTargetPosition = floatTargetPosition + (COUNTS_PER_FULL_REV * numOfRotations);
        magazine.setTargetPosition((int) floatTargetPosition);
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // CONTINUOUS CONTROL LOOP - keep updating until we reach target
        int tolerance = 5; // encoder counts tolerance
        while (opModeIsActive() && Math.abs((int)(floatTargetPosition) - magazine.getCurrentPosition()) > tolerance) {
            int currentPos = magazine.getCurrentPosition();

            // Calculate PID output based on current position
            double pid = controller.calculate(currentPos, (int)(floatTargetPosition));

            // Clamp power
            power = Math.max(-1.0, Math.min(1.0, pid));

            // Apply power
            magazine.setPower(power);

            // Small delay to prevent CPU overload
            sleep(10);
        }

        // Stop the motor when target is reached
        magazine.setPower(0);
        sleep(200); // Brief pause for stability
    }

        //magazine.setPower(1.0);
        /*while (Math.abs(floatTargetPosition - magazine.getCurrentPosition()) > 5) { // 5 is tolerance

            double command = control.update((int) floatTargetPosition, magazine.getCurrentPosition());
            command = Math.max(-1.0, Math.min(1.0, command));
            // assign motor the PID output
            //magazine.setPower(command);
        } */


    public void launchMotif (int aprilTagID) {
        float theoreticalSlot = topSlotNumber;
        float numRotationsRequired = 0f;
        String[] motif = new String[3];
        switch (aprilTagID) {
            case 21:
                motif[0] = "ARTIFACT_GREEN";
                motif[1] = "ARTIFACT_PURPLE";
                motif[2] = "ARTIFACT_PURPLE";
                break;
            case 22:
                motif[0] = "ARTIFACT_PURPLE";
                motif[1] = "ARTIFACT_GREEN";
                motif[2] = "ARTIFACT_PURPLE";
                break;
            case 23:
                motif[0] = "ARTIFACT_PURPLE";
                motif[1] = "ARTIFACT_PURPLE";
                motif[2] = "ARTIFACT_GREEN";
                break;
            default:
                telemetry.addData("APRILTAG ERROR", "no case");
                telemetry.update();
                sleep(500);
                // code block to execute if no case matches (optional)
        }

        for (int i = 0; i < 3; i++) {
            theoreticalSlot = topSlotNumber;
            numRotationsRequired = 0f;
            if (colors[(int)theoreticalSlot].equals(null)) {
                telemetry.addData("value in array equals null", "skipped");
                telemetry.update();
                sleep(500);
                break;
            }
            while ((opModeIsActive()) && (!colors[(int) theoreticalSlot].equals(motif[i]))) {
                numRotationsRequired += 1f;
                theoreticalSlot += 1f;
                if (theoreticalSlot >= 3f) {
                    theoreticalSlot = theoreticalSlot - 3f;
                }
                if (numRotationsRequired >= 3f) {
                    numRotationsRequired -= 3f;
                }
                if (numRotationsRequired == 2f) {
                    numRotationsRequired = -1f;
                }
                telemetry.addData(colors[(int) theoreticalSlot], motif[i]);
                telemetry.update();
            }
            /*if (numRotationsRequired == 2f) { //this makes it rotate in the other direction instead of rotating twice
                numRotationsRequired = -1f;
            }*/
            telemetry.addData("numRotationsRequired", numRotationsRequired);
            telemetry.update();
            fullRotation(numRotationsRequired, 0.4F, false);
                while (magazine.isBusy() && opModeIsActive()) {
                    //hold loop while function moving
                }
            colors[(int)theoreticalSlot] = "";
            telemetry.addData("launching", i);
            telemetry.addData("Bottom Slot", slotNumber);
            telemetry.addData("Top Slot", topSlotNumber);
            telemetry.addData("current color", colors[(int)theoreticalSlot]);
            telemetry.addData("Current motif", motif[i]);
            telemetry.addData("colors 0", colors[0]);
            telemetry.addData("colors 1", colors[1]);
            telemetry.addData("colors 2", colors[2]);
            telemetry.update();
            sleep(3000);//launch
            }

        }
    //}


    // Stop all motion;
    //leftDrive.setPower(0);
    //rightDrive.setPower(0);

    // Turn off RUN_TO_POSITION
    //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //sleep(250);   // optional pause after each move.
//*******************************************
//Taken from the TwoVisionPortals.java file. Not sure what these mean but these are just methods
//*******************************************

private void initAprilTag() {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;

    // First, create an AprilTagProcessor.Builder.
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    // Create each AprilTagProcessor by calling build.
    myAprilTagProcessor_1 = myAprilTagProcessorBuilder.build();
    //myAprilTagProcessor_2 = myAprilTagProcessorBuilder.build();
    Make_first_VisionPortal();
    Make_second_VisionPortal();
}

/**
 * Describe this function...
 */
private void Make_first_VisionPortal() {
    // Create a VisionPortal.Builder and set attributes related to the first camera.
    myVisionPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM_1) {
        // Use a webcam.
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
        // Use the device's back camera.
        myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Manage USB bandwidth of two camera streams, by adjusting resolution from default 640x480.
    // Set the camera resolution.
    myVisionPortalBuilder.setCameraResolution(new Size(320, 240));
    // Manage USB bandwidth of two camera streams, by selecting Streaming Format.
    // Set the stream format.
    myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
    // Add myAprilTagProcessor to the VisionPortal.Builder.
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor_1);
    // Add the Portal View ID to the VisionPortal.Builder
    // Set the camera monitor view id.
    myVisionPortalBuilder.setLiveViewContainerId(Portal_1_View_ID);
    // Create a VisionPortal by calling build.
    myVisionPortal_1 = myVisionPortalBuilder.build();
}

/**
 * Describe this function...
 */
private void Make_second_VisionPortal() {
    VisionPortal.Builder secondPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM_2) {
        // Use a webcam.
        secondPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
    } else {
        // Use the device's back camera.
        secondPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Manage USB bandwidth of two camera streams, by adjusting resolution from default 640x480.
    // Set the camera resolution.
    secondPortalBuilder.setCameraResolution(new Size(320, 240));
    // Manage USB bandwidth of two camera streams, by selecting Streaming Format.
    // Set the stream format.
    secondPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
    // Add myAprilTagProcessor to the VisionPortal.Builder.
    //myVisionPortalBuilder.addProcessor(myAprilTagProcessor_2);
    secondPortalBuilder.addProcessor(colorSensor);
    // Add the Portal View ID to the VisionPortal.Builder
    // Set the camera monitor view id.
    secondPortalBuilder.setLiveViewContainerId(Portal_2_View_ID);
    // Create a VisionPortal by calling build.
    myVisionPortal_2 = secondPortalBuilder.build();
}

/**
 * Describe this function...
 */
private void Toggle_camera_streams() {
    // Manage USB bandwidth of two camera streams, by turning on or off.
    if (gamepad1.dpad_down) {
        // Temporarily stop the streaming session. This can save CPU
        // resources, with the ability to resume quickly when needed.
        myVisionPortal_1.stopStreaming();
    } else if (gamepad1.dpad_up) {
        // Resume the streaming session if previously stopped.
        myVisionPortal_1.resumeStreaming();
    }
    if (gamepad1.dpad_left) {
        // Temporarily stop the streaming session. This can save CPU
        // resources, with the ability to resume quickly when needed.
        myVisionPortal_2.stopStreaming();
    } else if (gamepad1.dpad_right) {
        // Resume the streaming session if previously stopped.
        myVisionPortal_2.resumeStreaming();
    }
}

/**
 * Display info (using telemetry) for a recognized AprilTag.
 *
 * @return
 */
private String AprilTag_telemetry_for_Portal_1() {
    List<AprilTagDetection> myAprilTagDetections_1;
    AprilTagDetection thisDetection_1;

    // Get a list of AprilTag detections.
    myAprilTagDetections_1 = myAprilTagProcessor_1.getDetections();
    telemetry.addData("Portal 1 - # AprilTags Detected", JavaUtil.listLength(myAprilTagDetections_1));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection thisDetection_1_item : myAprilTagDetections_1) {
        thisDetection_1 = thisDetection_1_item;
        telemetry.addLine("First Apriltag ID (if 1+ detected, will be SKIPPED" + thisDetection_1.id);
        if (thisDetection_1.metadata != null) {  // Check first!
            return String.valueOf(thisDetection_1.id);
        }
        // Display info about the detection.
        //telemetry.addLine("");
        /* if (thisDetection_1.metadata != null) {
            telemetry.addLine("==== (ID " + thisDetection_1.id + ") " + thisDetection_1.metadata.name);
            //telemetry.addLine("XYZ " + JavaUtil.formatNumber(thisDetection_1.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.z, 6, 1) + "  (inch)");
            //telemetry.addLine("PRY " + JavaUtil.formatNumber(thisDetection_1.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.roll, 6, 1) + "  (deg)");
            //telemetry.addLine("RBE " + JavaUtil.formatNumber(thisDetection_1.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
        } else {
            //telemetry.addLine("==== (ID " + thisDetection_1.id + ") Unknown");
            return null; */
            //telemetry.addLine("Center " + JavaUtil.formatNumber(thisDetection_1.center.x, 6, 0) + "" + JavaUtil.formatNumber(thisDetection_1.center.y, 6, 0) + " (pixels)");
        }
    return null;
    }

//}

/**
 * Display info (using telemetry) for a recognized AprilTag.
 */
private void AprilTag_telemetry_for_Portal_2() {
    return;
    /*
    //List<AprilTagDetection> myAprilTagDetections_2;
    //AprilTagDetection thisDetection_2;

    // Get a list of AprilTag detections.
    //myAprilTagDetections_2 = myAprilTagProcessor_2.getDetections();
    myAprilTagDetections_2 = null; ///TO DISABLE THIS METHOD. IF YOU NEED IT GO BACK TO THE SAMPLE FILE
    telemetry.addLine("");
    telemetry.addData("Portal 2 - # AprilTags Detected", JavaUtil.listLength(myAprilTagDetections_2));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection thisDetection_2_item : myAprilTagDetections_2) {
        thisDetection_2 = thisDetection_2_item;
        // Display info about the detection.
        telemetry.addLine("");
        if (thisDetection_2.metadata != null) {
            telemetry.addLine("==== (ID " + thisDetection_2.id + ") " + thisDetection_2.metadata.name);
            telemetry.addLine("XYZ " + JavaUtil.formatNumber(thisDetection_2.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.z, 6, 1) + "  (inch)");
            telemetry.addLine("PRY " + JavaUtil.formatNumber(thisDetection_2.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.roll, 6, 1) + "  (deg)");
            telemetry.addLine("RBE " + JavaUtil.formatNumber(thisDetection_2.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
        } else {
            telemetry.addLine("==== (ID " + thisDetection_2.id + ") Unknown");
            telemetry.addLine("Center " + JavaUtil.formatNumber(thisDetection_2.center.x, 6, 0) + "" + JavaUtil.formatNumber(thisDetection_2.center.y, 6, 0) + " (pixels)");
        }
    }
 */ }

/**
 * Describe this function...
 */
private void AprilTag_telemetry_legend() {
    telemetry.addLine("");
    telemetry.addLine("key:");
    telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");
}

}



