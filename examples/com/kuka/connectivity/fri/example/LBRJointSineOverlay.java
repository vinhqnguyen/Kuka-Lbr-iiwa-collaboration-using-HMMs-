package com.kuka.connectivity.fri.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRIJointOverlay;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;

/**
 * Creates a FRI Session.
 */
public class LBRJointSineOverlay extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _hostName;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _hostName = "127.0.0.1";
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _hostName);
        friConfiguration.setSendPeriodMilliSec(4);
        FRISession friSession = new FRISession(friConfiguration);
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {

        }

        // move to start pose
        _lbr.move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0));

        // async move with overlay ...
        _lbr.moveAsync(ptp(Math.toRadians(-90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0)
                .setJointVelocityRel(0.2)
                .addMotionOverlay(jointOverlay)
                .setBlendingRel(0.1)
                );

        // ... blending into sync move with overlay
        _lbr.move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0)
                .setJointVelocityRel(0.2)
                .addMotionOverlay(jointOverlay)
                );

        // done
        friSession.close();
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final LBRJointSineOverlay app = new LBRJointSineOverlay();
        app.runApplication();
    }

}
