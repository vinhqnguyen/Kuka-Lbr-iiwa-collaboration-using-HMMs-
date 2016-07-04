package com.kuka.roboticsAPI.smartServo.samples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

/**
 * Very simple sample of Cartesian SmartServo Motion.
 * 
 * What you should learn in this sample:
 * <ul>
 * <li>Startup of a SmartServo Motion
 * <li>Send a sequence of cartesian setpoints to the controller
 * </ul>
 * 
 * @author schreiberg
 * 
 */
public class SmartServoSampleSimpleCartesian extends RoboticsAPIApplication
{
    // members
    private LBR _theLbr;
    private PhysicalObject _toolAttachedToLBR;

    @Override
    public void initialize()
    {
        // //////////////////////////////////////////////////////////////
        //
        // Locate the "first" Lightweight Robot in the system
        _theLbr = ServoMotionUtilities.locateLBR(getContext());
        //
        // ////////////////////////////////////////////////////////////////

        // ///////////////////////////////////////////////////////////////////////////////
        //
        // FIXME: Set proper Weights or use the plugin feature
        double[] translationOfTool =
        { 0, 0, 100 };
        double mass = 0;
        double[] centerOfMassInMillimeter =
        { 0, 0, 100 };
        _toolAttachedToLBR = ServoMotionUtilities.createTool(_theLbr,
                "SimpleJointMotionSampleTool", translationOfTool, mass,
                centerOfMassInMillimeter);

    }

    /**
     * Move to an initial Position WARNING: MAKE SHURE, THAT the pose is
     * collision free.
     */
    private void moveToInitialPosition()
    {
        _toolAttachedToLBR.move(
                ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
                        Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
        if (SmartServo.validateForImpedanceMode(_toolAttachedToLBR) != true)
        {
            System.out.println("validation of torque model failed - correct your mass property settings");
            System.out
                    .println("smartServo will be available for position controlled mode only, until validation is performed");
        }
    }

    /**
     * Sleep in between to emulate some computational effort.
     */
    int _milliSleepToEmulateComputationalEffort = 20;
    private int _numRuns = 10000;
    private double _amplitude = 50.;
    private double _freqency = 0.1;

    /**
     * Main Application Routine.
     */
    public void run()
    {
        moveToInitialPosition();

        boolean doDebugPrints = true;

        SmartServo aSmartServoMotion = new SmartServo(
                _theLbr.getCurrentJointPosition());

        aSmartServoMotion.useTrace(true);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(5e-3);

        System.out.println("Starting RealtimeMotion in Position Mode");
        _toolAttachedToLBR.moveAsync(aSmartServoMotion);

        // Fetch the Runtime of the Motion part
        ISmartServoRuntime theServoRuntime = aSmartServoMotion
                .getRuntime();

        Frame aFrame = theServoRuntime.getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());

        try
        {
            // do a cyclic loop
            // Do some timing...
            // in nanosec
            double omega = _freqency * 2 * Math.PI * 1e-9;
            long startTimeStamp = System.nanoTime();
            for (int i = 0; i < _numRuns; ++i)
            {

                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system
                theServoRuntime.updateWithRealtimeSystem();
                // Get the measured position in cartesian...
                Frame msrPose = theServoRuntime
                        .getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());

                if (doDebugPrints)
                {
                    System.out.println("cur cart goal" + aFrame);
                    System.out.println("cur destination Joints"
                            + theServoRuntime.getCurrentJointDestination());
                }

                // Do some Computation
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(_milliSleepToEmulateComputationalEffort);

                // do a cyclic loop

                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                // compute a new commanded position
                Frame destFrame = new Frame(aFrame);
                destFrame.setZ(_amplitude * Math.sin(sinArgument));
                // ////////////////////////////////////////////////////////////
                if (doDebugPrints)
                {
                    System.out.println("New CartGoal " + destFrame);
                    System.out.println(" LBR Position "
                            + _theLbr.getCurrentCartesianPosition(_theLbr
                                    .getFlange()));
                    System.out.println("Measured Cartesian Pose from Runtime "
                            + msrPose);
                }
                if ((i % 100) == 0)
                {
                    // Some internal values, which can be displayed
                    System.out.println(" simple Cartesian Test " + theServoRuntime.toString());
                }
                theServoRuntime.setDestination(destFrame, _toolAttachedToLBR.getDefaultMotionFrame());
            } // end for
        }
        catch (Exception e)
        {
            System.out.println(e);
            e.printStackTrace();
        }

        // /////////////////////////////////////////////////
        // Do or die: print statistics and parameters of the motion

        System.out.println(" simple Cartesian Test " + theServoRuntime.toString());
        // Stop the motion
        theServoRuntime.stopMotion();

    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main(String[] args)
    {
        SmartServoSampleSimpleCartesian app = new SmartServoSampleSimpleCartesian();
        app.runApplication();
    }
}
