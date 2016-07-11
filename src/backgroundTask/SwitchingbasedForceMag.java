package backgroundTask;


import java.io.FileNotFoundException;
import java.util.concurrent.TimeUnit;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import com.kuka.generated.ioAccess.DigOutIOGroup;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;


/**
 * Description: This program run cyclicly estimation of human's intention 
 * of rotation vs. translation based on Force magnitude.
 * Translation <= (F > f0 + delta_f)
 * Rotation    <= (F < f0)
 *  
 * Version: July 04, 2016
 * Author: Vinh Quang Nguyen (vinhnguyen.ac@gmail.com)
 */



public class SwitchingbasedForceMag extends RoboticsAPICyclicBackgroundTask {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private DigOutIOGroup DO;
	
	public static double f0, relay;
	public static int prediction_SF = 0; 
	public static int count_SF = 2;
	
	double F, fx, fy, fz, Tx, Ty, Tz;
	private PrintWriter writer;
	
	
	
	// Initialize KUKA controller
	public void initialize(){
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		initializeCyclic(0, 10, TimeUnit.MILLISECONDS,
				CycleBehavior.BestEffort);
		
        lbr = (LBR) ServoMotionUtilities.locateLBR(getContext());
        // FIXME: Set proper Weights or use the plugin feature

        DO = new DigOutIOGroup(kuka_Sunrise_Cabinet_1);
		
        // Initialize threshold and hysterisis for switching function
        f0 = 7.230;      //threshold
        relay = 5.33;   //hysterisis
		
        
        // Initialize writer
        try {
			writer = new PrintWriter("C:\\KRC\\ApplicationServer\\Git\\ManipulationTask\\Results_SF.txt", "UTF-8");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
	
	// Run cyclically
	public void runCyclic(){
		
		//==========================================================================================//
		// 1. Check the runCyclic() works? blink LED every 200 milisec.                             //
		//==========================================================================================//
		count_SF = count_SF + 1;
		if ((count_SF % 30 == 0) && (count_SF < 20000)){
			DO.setDigOut_5(!DO.getDigOut_5());
		}
		
		
		
		//==========================================================================================//
		// 2. Collect data every 40 milisec                                                         //
		//==========================================================================================//
		
		//	Collect data read from the controller
		ForceSensorData data = lbr.getExternalForceTorque(lbr.getFlange());  //make sure this force is described in moving frame, not fixed frame
		//JointPosition JointPos = lbr.getCurrentJointPosition();
		
		Vector force = data.getForce();

		// Do calibration
		fx = force.getX() - 3.6;
		fy = force.getY() + 3;
		fz = force.getZ() - 14.5;
		
		// Calculate force magnitude
		//F = Math.sqrt(Math.pow(fx, 2) + Math.pow(fy, 2) + Math.pow(fz, 2));
		F = Math.sqrt(Math.pow(fx, 2) + Math.pow(fy, 2));
		
		
		
		//==========================================================================================//
		// 3. Switching based force magnitude                                                       //
		//==========================================================================================//
		
		// Switching
		prediction_SF = Switch(F, f0, relay, prediction_SF); //Switch(double R, double T, double relay, double biasT, int oldPredict)
		
		// Output signal to LEDs. This is not necessary but it is easier for checking 
		// LED 3 is on: rotation
		// LED 4 is on: translation
		if (prediction_SF == 0) {
			DO.setDigOut_7(false);
			DO.setDigOut_8(true);   //translation
		}
		
		else {
			DO.setDigOut_7(true);   //rotation
			DO.setDigOut_8(false); 
		}
		
		
		
		//==============================================================================================//
		// 4. Write the results into text file for later evaluation                                     //
		//==============================================================================================//		
		if (count_SF < 20000){
		writer.println(System.currentTimeMillis() + " " + fx + " " + fy + " " + fz + " " + F + " " + prediction_SF);
		}
		
	}
	
	
	
	public static int Switch(double F, double f0, double relay, int oldPredict){
		
		int currPredict = 0;
		
		if (oldPredict == 0) {
			if (F < f0) currPredict = 1; //Rotation
		}
		else currPredict = 0;
		
		if (oldPredict == 1)
		if (F > f0 + relay){
			currPredict = 0; //Translation
		}
		else currPredict = 1;
		
		return currPredict;
	}
	
}
