package exercises;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.Motion;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.RobotMotion;
import robot.ExecutionController;
import robot.SunriseConnector;
import utility.DataHandler;
import utility.FileLogger;
import utility.SingleInstanceChecker;
import utility.FileLogger.Fields;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;

import java.util.Vector;

public class Task_04_JointAndTaskSpace extends RoboticsAPIApplication {
	
	public Task_04_JointAndTaskSpace(RoboticsAPIContext context){
		super(context); 
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();
		
		// initialization
		Task_04_JointAndTaskSpace app = new Task_04_JointAndTaskSpace(RoboticsAPIContext.createFromResource(Task_04_JointAndTaskSpace.class, "RoboticsAPI.config.xml")); 
		SunriseConnector.initialize(app);

		app.run();
		
		ExecutionController.waitForAllMotionsFinished();
	}

	LBR robot;
	Tool tool;

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		System.out.println("Starting " + this.getClass().getSimpleName());

		double blendingRadiusCart = 10;
		double jointVelocity = 0.2;

		//Frame F0 = DataHandler.loadFrame("F0_Home");
		Frame F1 = DataHandler.loadFrame("F1");
		Frame F2 = DataHandler.loadFrame("F2");
		Frame F3 = DataHandler.loadFrame("F3");
		Frame F4 = DataHandler.loadFrame("F4");

		JointPosition J0 = DataHandler.loadJointPos("J0");
		JointPosition J1 = DataHandler.loadJointPos("J1");
		JointPosition J2 = DataHandler.loadJointPos("J2");
		JointPosition J3 = DataHandler.loadJointPos("J3");
		JointPosition J4 = DataHandler.loadJointPos("J4");

		/** ----- create a motion batch with PTP movements to joint positions ----- */
		/** add Cartesian blending with 10 mm and set joint velocity to 0.2  */
		Vector<RobotMotion<?>> motionVectorLin = new Vector<RobotMotion<?>>();
		motionVectorLin.add(BasicMotions.lin(F1));
		motionVectorLin.add(BasicMotions.lin(F2));
		motionVectorLin.add(BasicMotions.lin(F3));
		motionVectorLin.add(BasicMotions.lin(F3));

		Vector<RobotMotion<?>> motionVectorPtP = new Vector<RobotMotion<?>>();
		motionVectorPtP.add(BasicMotions.ptp(F1));
		motionVectorPtP.add(BasicMotions.ptp(F2));
		motionVectorPtP.add(BasicMotions.ptp(F3));
		motionVectorPtP.add(BasicMotions.ptp(F3));

		RobotMotion<?>[] motionArray = motionVectorLin.toArray(new RobotMotion<?>[motionVectorLin.size()]);
		RobotMotion<?>[] motionArrayPtP = motionVectorPtP.toArray(new RobotMotion<?>[motionVectorPtP.size()]);
		MotionBatch mbLin = new MotionBatch(motionArray);
		MotionBatch mbPtP = new MotionBatch(motionArrayPtP);
		mbLin.setBlendingCart(10);
		mbLin.setJointVelocityRel(0.2);

		mbPtP.setBlendingCart(10);
		mbPtP.setJointVelocityRel(0.2);

		/** use a sync move to the first (joint-)position, then start logging and execute the motion batch */

		/** ----- create a motion batch with PTP movements to frames ----- */
		/** add Cartesian blending with 10 mm and set joint velocity to 0.2 */

		tool.move(BasicMotions.ptp(F1));
		FileLogger.startLogging("log_PTP_jp", Fields.TIME, Fields.TRANSLATIONAL_DISTANCE, Fields.ROTATIONAL_DISTANCE,
				Fields.JOINT_SPACE_DISTANCE);
		tool.move(mbPtP);
		FileLogger.stopLogging();
		

		/** ----- create a motion batch with LIN movements to frames ----- */
		/** add Cartesian blending with 10 mm and set joint velocity to 0.2 */

		tool.move(BasicMotions.ptp(F1));
		FileLogger.startLogging("log_LIN_jp", Fields.TIME, Fields.TRANSLATIONAL_DISTANCE, Fields.ROTATIONAL_DISTANCE,
				Fields.JOINT_SPACE_DISTANCE);
		tool.move(mbLin);
		FileLogger.stopLogging();

		/** wait until movements are finished [if there are async movements] */
		ExecutionController.waitForAllMotionsFinished();
		System.exit(0);
	}
}
