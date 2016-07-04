package com.kuka.roboticsAPI.smartServo.samples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

/**
 * Very simple sample of JointSpecific SmartServo Motion.
 * 
 * What you should learn in this lesson:
 * 
 * <ul>
 * <li>Activation of a smart servo motion in default control mode
 * <li>Sending a sequence joint specific setpoints
 * <li>the Action of the StatisticTimer for evaluating bandwith/Timing Issues
 * </ul>
 * 
 * @author schreiberg
 * 
 */
public class SmartServoSampleSimpleJointMotion extends RoboticsAPIApplication
{
    // members
    private LBR _theLbr;
    /**
     * Will be initialized from the routine "createTool()".
     * 
     * IMPORTANT NOTE: Set the load mass properly
     */
    private PhysicalObject _toolAttachedToLBR;
    private ISmartServoRuntime theSmartServoRuntime = null;
    private int _count = 0;

    @Override
    public void initialize()
    {
        getContext().dumpDevices();

        // Locate the "first" Lightweight Robot in the system
        _theLbr = ServoMotionUtilities.locateLBR(getContext());
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
    public void moveToInitialPosition()
    {
        _toolAttachedToLBR.move(
                ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
                        Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
        /*
         * 
         * For Completeness Sake, the validation is performed here Even it would
         * not be necessary within this sample.
         * 
         * As long, as you'd remain within position control, the validation is
         * not necessary ... but, lightweight robot without ImpedanceControl is
         * a car without fuel...
         * 
         * Note: The Validation itself justifies, that in this very time
         * instance, the load parameter setting was sufficient. This does not
         * mean by far, that the parameter setting is valid in the sequel or
         * lifetime of this program
         */
        try
        {
            if (!SmartServo.validateForImpedanceMode(_toolAttachedToLBR))
            {
                System.out
                        .println("Validation of Torque Model failed - correct your mass property settings");
                System.out
                        .println("SmartServo will be available for position controlled mode only, until validation is performed");
            }
        }
        catch (IllegalStateException e)
        {
            System.out.println("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
    }

    // Sleep in between
    private int _milliSleepToEmulateComputationalEffort = 10;//000;
    private int _numRuns = 10000;
    private double _amplitude = 0.2;
    private double _freqency = 0.1;
    private int steps = 0;

    @Override
    public void run()
    {

        moveToInitialPosition();

        boolean doDebugPrints = false;

        JointPosition initialPosition = new JointPosition(
                _theLbr.getCurrentJointPosition());
        SmartServo aSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 20% of systems abilities
        aSmartServoMotion.setJointAccelerationRel(0.2);
        aSmartServoMotion.setJointVelocityRel(0.2);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);

        System.out.println("Starting SmartServo in Position Mode");
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(aSmartServoMotion);

        // Fetch the Runtime of the Motion part
        theSmartServoRuntime = aSmartServoMotion.getRuntime();

        // create an JointPosition Instance, to play with
        JointPosition destination = new JointPosition(
                _theLbr.getJointCount());
        System.out.println("start loop");
        // For Roundtrip time measurement...
        StatisticTimer timing = new StatisticTimer();
        try
        {
            // do a cyclic loop
            // Refer to some timing...
            // in nanosec
            double omega = _freqency * 2 * Math.PI * 1e-9; // freqency * ...
            long startTimeStamp = System.nanoTime();
            for (steps = 0; steps < _numRuns; ++steps)
            {
                // Timing - draw one step
                OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(_milliSleepToEmulateComputationalEffort);
                theSmartServoRuntime.updateWithRealtimeSystem();
                // Get the measured position
                JointPosition curMsrJntPose = theSmartServoRuntime
                        .getAxisQMsrOnController();

                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                for (int k = 0; k < destination.getAxisCount(); ++k)
                {
                    destination.set(k, Math.sin(sinArgument)
                            * _amplitude + initialPosition.get(k));
                    if (k > 5)
                    {
                        destination.set(k, initialPosition.get(k));
                    }
                }
                theSmartServoRuntime.setDestination(destination);

                // ////////////////////////////////////////////////////////////
                if (steps % 100 == 0)
                {
                    System.out.println("Step " + steps + " New Goal "
                            + destination);
                    System.out.println("fine ipo finished" + theSmartServoRuntime.isDestinationReached());
                    System.out.println("ipo state" + theSmartServoRuntime.getFineIpoState());
                    System.out.println("remaining time" + theSmartServoRuntime.getRemainingTime());
                    System.out.println(" LBR Position "
                            + _theLbr.getCurrentJointPosition());
                    System.out.println(" Measured LBR Position "
                            + curMsrJntPose);
                    if (doDebugPrints)
                    {
                        // Some internal values, which can be displayed
                        System.out.println(" simple Joint Test - step " + steps + theSmartServoRuntime.toString());

                    }
                }
                // Overall timing end
                aStep.end();

            } // end for
        }
        catch (Exception e)
        {
            System.out.println(e);
            e.printStackTrace();
        }
        ThreadUtil.milliSleep(1000);
        // /////////////////////////////////////////////////
        // Do or die: print statistics and parameters of the motion
        System.out.println("Displaying final states after loop ");
        System.out.println(getClass().getName() + theSmartServoRuntime.toString());
        // Stop the motion
        theSmartServoRuntime.stopMotion();
        System.err.println(_count + "times was the destination reached.");
        System.err.println("Statistic Timing of Overall Loop " + timing);
        if (timing.getMeanTimeMillis() > 150)
        {
            System.out
                    .println("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            System.out
                    .println("Under Windows, you should play with the registry, see the e.g. the SmartServo Class javaDoc for details");
        }
    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main(String[] args)
    {
        SmartServoSampleSimpleJointMotion app = new
                SmartServoSampleSimpleJointMotion();
        app.runApplication();
    }
}
