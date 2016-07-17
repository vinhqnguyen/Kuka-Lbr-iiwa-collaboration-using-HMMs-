package backgroundTask;


import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.Reader;
import java.util.concurrent.TimeUnit;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.List;

import be.ac.ulg.montefiore.run.jahmm.Hmm;
import be.ac.ulg.montefiore.run.jahmm.ObservationVector;
import be.ac.ulg.montefiore.run.jahmm.OpdfMultiGaussianFactory;
import be.ac.ulg.montefiore.run.jahmm.io.FileFormatException;
import be.ac.ulg.montefiore.run.jahmm.io.ObservationSequencesReader;
import be.ac.ulg.montefiore.run.jahmm.io.ObservationVectorReader;
import be.ac.ulg.montefiore.run.jahmm.learn.BaumWelchScaledLearner;
import be.ac.ulg.montefiore.run.jahmm.learn.KMeansLearner;

import com.kuka.generated.ioAccess.DigOutIOGroup;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;


/**
 * Implementation of a cyclic background task.
 * Author: Vinh Quang Nguyen (vinhnguyen.ac@gmail.com)
 * Description: This program run cyclicly HMMs to estimate the person's intention
 * base on the recorded haptic data.
 * Procedure: 
 * 1. Initialize: train HMMs, and declare a observation sequence length of 25, dimension 3
 * 2. Runcyclic: Getting data force at end effector, then running HMMs every 40 milisec
 * 
 *  Data: QianDong's data in both directions (CW and CCW), obs = (Ty, Tz, Fx)
 *  Version: July 07, 2016
 */
public class Running4HMMs extends RoboticsAPICyclicBackgroundTask {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private DigOutIOGroup DO;
	
	private static Hmm<ObservationVector > Hmm_Rot_inRot, Hmm_Tra_inRot, Hmm_Rot_inTra, Hmm_Tra_inTra;
	private static List<ObservationVector> obsSeq = new ArrayList<ObservationVector>();
	
	long period = 40;
	public static int prediction = 0, prediction_old = 0, temp = 0, temp_old = 0; 
	public static int count = 2;
	
    static int nbStates_inRot = 3, nbStates_inTra = 3;
    static int dimension = 3;
    static int len = 25;
    public static double bias = 0, relay = 0;
    
	double Fx, Fy, Fz, Tx, Ty, Tz, R, T;
	private PrintWriter writer;
	private ObservationVector obs;
	private ForceSensorData data;
	private JointPosition JointPos;
	private Vector torque, force;
	
	public void initialize(){
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		initializeCyclic(0, period, TimeUnit.MILLISECONDS,
				CycleBehavior.BestEffort);
		
        lbr = (LBR) ServoMotionUtilities.locateLBR(getContext());
        // FIXME: Set proper Weights or use the plugin feature

        DO = new DigOutIOGroup(kuka_Sunrise_Cabinet_1);
		
		
        // 1. Train HMMs here
        try {
			Hmm_Rot_inRot = trainHMM_inRot("C:\\KRC\\ApplicationServer\\Git\\ManipulationTask\\dataTrainRot_inRot.txt");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		try {
			Hmm_Tra_inRot = trainHMM_inRot("C:\\KRC\\ApplicationServer\\Git\\ManipulationTask\\dataTrainTra_inRot.txt");
		} catch (IOException e){
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
        try {
			Hmm_Rot_inTra = trainHMM_inTra("C:\\KRC\\ApplicationServer\\Git\\ManipulationTask\\dataTrainRot_inTra.txt");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		try {
			Hmm_Tra_inTra = trainHMM_inTra("C:\\KRC\\ApplicationServer\\Git\\ManipulationTask\\dataTrainTra_inTra.txt");
		} catch (IOException e){
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		// 2. Initialize observation sequences
		double [] dataPoint2 = new double[dimension];
		for (int i=0; i<dimension; i++)
		{
				dataPoint2[i] = 0;
		}
		
		for (int i = 0; i <len; i++) {
		    ObservationVector obs = new ObservationVector(dataPoint2);
		    obsSeq.add(obs);
		} 
		
        
		// 3. Initialize writer for recording results
		try {
			writer = new PrintWriter("C:\\KRC\\ApplicationServer\\Git\\ManipulationTask\\Results.txt", "UTF-8");
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
		count = count + 1;
		if ((count % 20 == 0) && (count < 12000)){
			DO.setDigOut_1(!DO.getDigOut_1());
		}
		
		
		
		//==========================================================================================//
		// 2. Collect data every 40 milisec, then run HMMs to get the results                       //
		//==========================================================================================//
		//Collect data read from the controller
		data = lbr.getExternalForceTorque(lbr.getFlange());  //make sure this force is described in moving frame, not fixed frame
		JointPos = lbr.getCurrentJointPosition();
		//double A = Pose1.getAlphaRad();
		double J7 = JointPos.get(6);
		
		force = data.getForce();
		torque = data.getTorque();
		
		//calibration
		Tx = torque.getX() + 0.85; 
		if (J7 < 0) Tx += (0.75*J7); //Tx is not used
		
		Ty = torque.getY();
		if (J7 > -1.3) Ty = Ty - 0.8*J7;
	    if (J7 < 0 && J7 > -1.3) 
	        Ty = Ty - 0.3*(J7);
	    if (J7 < -1.3) 
	        Ty = Ty + 0.3*(J7) + 0.6;
	    
		Tz = (torque.getZ());
		
		Fx = force.getX() - 3.6;
		
		
		
		//==========================================================================================//
		// 3. Run HMMs                                                                              //
		//==========================================================================================//
	    obs = new ObservationVector(new double[]{Ty, Tz, Fx});
	    obsSeq.remove(0);
	    obsSeq.add(obs);
	    
	    if(temp_old == 1) //in rotation mode
	    {
		R = Hmm_Rot_inRot.lnProbability(obsSeq);
		T = Hmm_Tra_inRot.lnProbability(obsSeq);
	    }
	    
	    else{
		R = Hmm_Rot_inTra.lnProbability(obsSeq);
		T = Hmm_Tra_inTra.lnProbability(obsSeq);
	    }
	    
		
		//Compare the logliks
		if (R > T) temp = 1;
		else temp = 0;
		
		
		//switching when previous and current prediction is the same
		if (temp == temp_old) 
			prediction = temp;
		else prediction = prediction_old;
		
		
		//only running HMMs if the person try to move the object
		//if(Math.abs(Tz) < 1) prediction = prediction_old;
		
		prediction_old = prediction;
		temp_old = temp;
		
		
		//Output the prediction to see on LEDs
		//LED 3 is on: rotation
		//LED 4 is on: translation
		if (prediction == 0) {
			DO.setDigOut_3(false);
			DO.setDigOut_4(true);   //translation
		}
		
		else {

			DO.setDigOut_3(true);   //rotation
			DO.setDigOut_4(false); 
		}
		
		
		
		
		//==============================================================================================//
		// 4. Write the results into text file for later evaluation                                     //
		//==============================================================================================//
		if (count < 12000){
		writer.println(System.currentTimeMillis() + " " + Ty + " " + Tz + " " + Fx + " " + prediction + " " + R + " " + T);
		}
		
	}
	
	
	public static Hmm <ObservationVector>  trainHMM_inRot (String fileName)throws IOException, FileFormatException{
		
		Hmm <ObservationVector> bwHmm;
		
		
		//1. Reading data from files
		Reader readerTrain = null;
		
	    readerTrain = new FileReader (fileName);
			
		List <List <ObservationVector >> seqsTrain =   //data train
				ObservationSequencesReader.
				readSequences (new ObservationVectorReader (),  
				readerTrain);

		readerTrain.close ();
		
		
		//2. Learning and testing with HMM Translation
		System.out.println("Learning");
		KMeansLearner <ObservationVector > kml =
				new KMeansLearner <ObservationVector >(nbStates_inRot,
				new OpdfMultiGaussianFactory (3), seqsTrain);
		
		Hmm <ObservationVector> initHmm = kml.learn ();
		
		/*
		//using BaumWelchLearner
		BaumWelchLearner bwl_Tra = new BaumWelchLearner();
		bwHmm = bwl_Tra.learn(initHmm, seqsTrain);
		for (int i = 0; i < 5; i++) {
		    bwHmm = bwl_Tra.iterate(bwHmm,seqsTrain);
		} */
		
		
		//3. Using Scaled BaumWelchLearner
		BaumWelchScaledLearner bwsl = new BaumWelchScaledLearner();
		bwHmm = bwsl.learn(initHmm, seqsTrain);
		
		return bwHmm;
	
	}
	
	public static Hmm <ObservationVector>  trainHMM_inTra (String fileName)throws IOException, FileFormatException{
		
		Hmm <ObservationVector> bwHmm;
		
		
		//1. Reading data from files
		Reader readerTrain = null;
		
	    readerTrain = new FileReader (fileName);
			
		//Translation data
		List <List <ObservationVector >> seqsTrain =   //data train
				ObservationSequencesReader.
				readSequences (new ObservationVectorReader (),  
				readerTrain);

		readerTrain.close ();
		
		
		//2. Learning and testing with HMM Translation
		System.out.println("Learning");
		KMeansLearner <ObservationVector > kml =
				new KMeansLearner <ObservationVector >(nbStates_inTra,
				new OpdfMultiGaussianFactory (3), seqsTrain);
		
		Hmm <ObservationVector> initHmm = kml.learn ();
		
		
		/*
		//using BaumWelchLearner
		BaumWelchLearner bwl_Tra = new BaumWelchLearner();
		bwHmm = bwl_Tra.learn(initHmm, seqsTrain);
		for (int i = 0; i < 5; i++) {
		    bwHmm = bwl_Tra.iterate(bwHmm,seqsTrain);
		} */
		
		
		//3. Using Scaled BaumWelchLearner
		BaumWelchScaledLearner bwsl = new BaumWelchScaledLearner();
		bwHmm = bwsl.learn(initHmm, seqsTrain);
		
		return bwHmm;
	
	}
	
}
