package exercises;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.persistenceModel.templateModel.FrameTemplate;
import robot.ExecutionController;
import robot.SunriseConnector;
import utility.DataHandler;
import utility.FileLogger;
import utility.SingleInstanceChecker;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

public class Task_08_Wireloop extends RoboticsAPIApplication {

	public Task_08_Wireloop(RoboticsAPIContext context) {
		super(context);
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();
		
		// initialization
		Task_08_Wireloop app = new Task_08_Wireloop(
				RoboticsAPIContext.createFromResource(Task_08_Wireloop.class, "RoboticsAPI.config.xml"));
		SunriseConnector.initialize(app);
		
		SunriseConnector.createInfoGui();
		
		SunriseConnector.getControlGui().setNextFrameName("wireloop_start");

		app.run();

		ExecutionController.waitForAllMotionsFinished();

		SunriseConnector.stopInfoGui();
		System.exit(0);
	}

	/**
	 * This class can be used to track the movement direction during the exploration. It can be set to POSITIVE and
	 * NEGATIVE and be toggled between both. Using value() returns the 1 or -1, respectively.
	 * 
	 * @author seid_da
	 *
	 */
	public static class Direction {

		public static final int POSITIVE = 1;
		public static final int NEGATIVE = -1;

		private int value;

		private Direction(int value) {
			this.value = value;
		}

		/**
		 * Create a new Direction instace with POSITIVE value.
		 * 
		 * @return Direction instance with value() returning 1.
		 */
		public static Direction POSITIVE() {
			return new Direction(POSITIVE);
		}

		/**
		 * Create a new Direction instace with NEGATIVE value.
		 * 
		 * @return Direction instance with value() returning -1.
		 */
		public static Direction NEGATIVE() {
			return new Direction(NEGATIVE);
		}

		/**
		 * Toggle the value of this Direction instance between 1 and -1.
		 */
		public void invert() {
			this.value *= -1;
		}

		/**
		 * Set the value of this Direction instance to 1.
		 */
		public void setPositive() {
			this.value = 1;
		}

		/**
		 * Set the value of this Direction instance to -1.
		 */
		public void setNegative() {
			this.value = -1;
		}

		/**
		 * Returns the current direction as an integer factor (1 or -1).
		 * 
		 * @return Signed int representing the direction.
		 */
		public final int value() {
			return value;
		}

		/**
		 * Returns the String representation of the current direction.
		 */
		public String toString() {
			return "" + value;
		}
	}

	private Direction xDirection = Direction.POSITIVE();
	private Direction yDirection = Direction.POSITIVE();

	private final double lowCartVel = 100;
	private final double highCartVel = 200;

	private final double moveUpDownDistance = 30;

	private Frame wireloopStart = null;

	private CartesianImpedanceControlMode cicm;

	LBR robot;
	Tool tool;
	ObjectFrame toolFrame;

	private List<Transformation> collisionFreeTrafos = new ArrayList<Transformation>();

	private Frame addToFrame(Frame oldFrame, Direction x, Direction y, int stepSize) {
		Frame newFrame = new Frame();
		newFrame = oldFrame.copyWithRedundancy();
		newFrame.setX(newFrame.getX() + stepSize*x.value());
		newFrame.setY(newFrame.getY() + stepSize*y.value());
		return newFrame;
	}

	private void moveUp() {
		RelativeLIN motion = BasicMotions.linRel(0, 0, moveUpDownDistance, robot.getRootFrame());
		tool.move(motion.setCartVelocity(highCartVel));
	}

	private void moveDown() {
		RelativeLIN motion = BasicMotions.linRel(0, 0, -moveUpDownDistance, robot.getRootFrame());
		tool.move(motion.setCartVelocity(highCartVel));
	}

	private double mapJointLimits(Direction xDirection, double currAx7, double incrementalAx7, double upperAxisLimit, double lowerAxisLimit) {
		double goalAx7 = currAx7 + incrementalAx7;
		Transformation currFrame = robot.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).transformationFromWorld();

		if (goalAx7 > (upperAxisLimit-2*0.174533)) {
			xDirection.invert();
			Transformation upMotion = Transformation.ofTranslation(0, 0, moveUpDownDistance);
			collisionFreeTrafos.add(upMotion.compose(currFrame));
			moveUp();

			Transformation rotMotion = Transformation.ofRad(0, 0, 0, -Math.PI, 0, 0);
			collisionFreeTrafos.add(rotMotion.compose(currFrame));
			tool.move(BasicMotions.linRel(0,0,0, -Math.PI, 0,0));

			currFrame = robot.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).transformationFromWorld();
			collisionFreeTrafos.add(rotMotion.compose(currFrame));
			tool.move(BasicMotions.linRel(0,0,0, -Math.PI/2, 0,0));

			currFrame = robot.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).transformationFromWorld();
			Transformation downMotion = Transformation.ofTranslation(0, 0, -moveUpDownDistance);
			collisionFreeTrafos.add(downMotion.compose(currFrame));
			moveDown();
		} else if (goalAx7 < (lowerAxisLimit+2*0.174533)) {
			xDirection.invert();
			Transformation upMotion = Transformation.ofTranslation(0, 0, moveUpDownDistance);
			collisionFreeTrafos.add(upMotion.compose(currFrame));
			moveUp();

			Transformation rotMotion = Transformation.ofRad(0, 0, 0, Math.PI, 0, 0);
			collisionFreeTrafos.add(rotMotion.compose(currFrame));
			tool.move(BasicMotions.linRel(0,0,0, Math.PI, 0,0));

			currFrame = robot.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).transformationFromWorld();
			collisionFreeTrafos.add(rotMotion.compose(currFrame));
			tool.move(BasicMotions.linRel(0,0,0, Math.PI/2, 0,0));

			currFrame = robot.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).transformationFromWorld();
			Transformation downMotion = Transformation.ofTranslation(0, 0, -moveUpDownDistance);
			collisionFreeTrafos.add(downMotion.compose(currFrame));
			moveDown();
		}

		return incrementalAx7;
	}

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		toolFrame = tool.getDefaultMotionFrame();
		System.out.println("Starting " + this.getClass().getSimpleName());

		//JointPosition above_wireloop = DataHandler.loadJointPos("J11");
		//JointPosition start_wireloop = DataHandler.loadJointPos("J12");
		Frame start_wireloop = DataHandler.loadFrame("F11");
		Frame above_wireloop = DataHandler.loadFrame("F11");
		above_wireloop.setZ(above_wireloop.getZ() + 30);

		//Frame start_wireloop_2 = DataHandler.loadFrame("F1");
		/** load start position [perviously teached] and define starting direction */
		//wireloopStart = DataHandler.loadFrame("wireloop_start").setBetaRad(0).setGammaRad(Math.PI);

		// xDirection = Direction.NEGATIVE();
		xDirection = Direction.POSITIVE();

		/** configure impedance control mode */
		cicm = new CartesianImpedanceControlMode();
		cicm.parametrize(CartDOF.TRANSL).setStiffness(2500);
		cicm.parametrize(CartDOF.ROT).setStiffness(300);

		/** for safety, move up */
		try {
			moveUp();
		} catch (Exception e) {
			SunriseConnector.goFront();
		}

		/** move to start position in two steps: first to the position 5 cm above the start and then vertical */
		//Frame aboveWireloopStart = new Frame(Transformation.ofTranslation(0, 0, 50)
		//		.compose(wireloopStart.getTransformationProvider().getTransformation()));
		tool.move((BasicMotions.ptp(above_wireloop).setJointVelocityRel(0.5)));
		tool.move(BasicMotions.ptp(start_wireloop).setJointVelocityRel(0.5));

		int stepSize = 10;
		Transformation trafo = Transformation.IDENTITY;

		double upperJointLimit7 = robot.getJointLimits().getMaxJointPosition().get(6);
		double lowerJointLimit7 = robot.getJointLimits().getMinJointPosition().get(6);
		int i = 0;
		do{
			i = i+1;
			ForceCondition condX = ForceCondition.createNormalForceCondition(null, CoordinateAxis.X, 5);
			ForceCondition condY = ForceCondition.createNormalForceCondition(null, CoordinateAxis.Y, 5);

			Frame currFrame = robot.getCurrentCartesianPosition(tool.getDefaultMotionFrame());
			collisionFreeTrafos.add(trafo.compose(currFrame.transformationFromWorld()));
			IMotionContainer mc = tool.move(BasicMotions.linRel(trafo, currFrame).setJointVelocityRel(0.5).breakWhen(condX).breakWhen(condY));
			double jointAngle7 = robot.getCurrentJointPosition().get(6);

			if (mc.hasFired(condX) || mc.hasFired(condY)) {
				Vector force = robot.getExternalForceTorque(toolFrame).getForce();
				System.out.println("Collision");
				Transformation oldTrafo = Transformation.of(trafo);
				if (abs(force.getY()) < 1) {
					if (force.getX() < 0) {
						trafo = Transformation.ofTranslation((double) -stepSize /2, 0, 0);
					} else {
						trafo = Transformation.ofTranslation((double) stepSize /2, 0, 0);
					}
					Transformation collidingTrafo = collisionFreeTrafos.removeLast();
					collisionFreeTrafos.add(trafo.compose(collidingTrafo));
					tool.move(BasicMotions.linRel(trafo, currFrame).setJointVelocityRel(0.5));
				}else if(abs(force.getX()) < 1){
					if (force.getY() < 0) {
						trafo = Transformation.ofTranslation(0, (double) (-stepSize) /2, 0);
					} else {
						trafo = Transformation.ofTranslation(0, (double) (stepSize) /2, 0);
					}
					Transformation collidingTrafo = collisionFreeTrafos.removeLast();
					collisionFreeTrafos.add(trafo.compose(collidingTrafo));
					tool.move(BasicMotions.linRel(trafo, currFrame).setJointVelocityRel(0.5));
				}else
				{
					if (force.getY() < 0) {
						// Left Bend
						System.out.println("Left Forces:");
						System.out.println(force.getX());
						System.out.println(force.getY());
						//double alpha = mapJointLimits(xDirection, jointAngle7, Math.PI-Math.atan(force.getX()/force.getY()), upperJointLimit7 , lowerJointLimit7);
						double alpha = mapJointLimits(xDirection, jointAngle7, -xDirection.value()*Math.PI/8, upperJointLimit7, lowerJointLimit7);//mapJointLimits(xDirection, jointAngle7, Math.PI-Math.atan(force.getX()/force.getY()), upperJointLimit7 , lowerJointLimit7);

						trafo = Transformation.ofRad(0, 0, 0, alpha,  0, 0);
						Transformation collidingTrafo = collisionFreeTrafos.removeLast();
						Transformation backwardMotion = Transformation.ofTranslation(-xDirection.value()*stepSize/2, 0, 0);
						collisionFreeTrafos.add(backwardMotion.compose(collidingTrafo));
						tool.move(BasicMotions.linRel(-xDirection.value()*stepSize/2, 0, 0, 0, 0, 0).setJointVelocityRel(0.5));
					}else{
						System.out.println("Right Forces:");
						// Right Bend
						System.out.println(force.getX());
						System.out.println(force.getY());
						//double alpha = mapJointLimits(xDirection, jointAngle7, Math.atan(force.getX() / force.getY()), upperJointLimit7, lowerJointLimit7);
						double alpha = mapJointLimits(xDirection, jointAngle7, xDirection.value()*Math.PI/8, upperJointLimit7, lowerJointLimit7);// mapJointLimits(xDirection, jointAngle7, Math.PI-Math.atan(force.getX()/force.getY()), upperJointLimit7 , lowerJointLimit7);

						Transformation collidingTrafo = collisionFreeTrafos.removeLast();
						Transformation backwardMotion = Transformation.ofTranslation(-xDirection.value()*stepSize/2, 0, 0);
						collisionFreeTrafos.add(backwardMotion.compose(collidingTrafo));

						trafo = Transformation.ofRad(0, 0, 0, alpha, 0, 0);
						tool.move(BasicMotions.linRel(-xDirection.value()*stepSize/2,0, 0, 0, 0, 0).setJointVelocityRel(0.5));
					}
				}
			}else {
				trafo = Transformation.ofTranslation(xDirection.value()*stepSize, 0, 0);
				System.out.println("No collision");
			}
			System.out.println(robot.getCurrentCartesianPosition(toolFrame).distanceTo(start_wireloop));
		}while (robot.getCurrentCartesianPosition(toolFrame).distanceTo(start_wireloop) > 10 || i < 2);

		i = 0;
		tool.move(BasicMotions.ptp(start_wireloop).setJointVelocityRel(0.5));

		FileLogger.startLogging("log_wireloop_test", FileLogger.Fields.TIME, FileLogger.Fields.TRANSLATIONAL_DISTANCE, FileLogger.Fields.ROTATIONAL_DISTANCE,
				FileLogger.Fields.JOINT_SPACE_DISTANCE, FileLogger.Fields.MEASURED_TORQUES, FileLogger.Fields.JOINT_POSITION, FileLogger.Fields.TRANSLATION, FileLogger.Fields.ORIENTATION,FileLogger.Fields.EXTERNAL_TCP_FORCES_TORQUES);
		while (i<collisionFreeTrafos.size()) {
			Frame f = new Frame(robot.getRootFrame(), collisionFreeTrafos.get(i));
			toolFrame.move(BasicMotions.lin(f));
			i++;
		}
		FileLogger.stopLogging();
	}
}
