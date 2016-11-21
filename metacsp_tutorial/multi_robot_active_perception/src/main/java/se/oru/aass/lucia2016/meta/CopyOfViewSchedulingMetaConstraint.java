package se.oru.aass.lucia2016.meta;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.framework.VariableOrderingH;
import org.metacsp.framework.meta.MetaConstraint;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.meta.symbolsAndTime.Schedulable;
import org.metacsp.meta.symbolsAndTime.Schedulable.PEAKCOLLECTION;
import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.Bounds;

import se.oru.aass.lucia2016.multi.ViewConstraintSolver;
import se.oru.aass.lucia2016.multi.ViewVariable;





import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.MultiPolygon;
import com.vividsolutions.jts.geom.Point;

public class CopyOfViewSchedulingMetaConstraint extends MetaConstraint{

	/**
	 * Iran
	 */
	private static final long serialVersionUID = -5291826562604152461L;
	private HashMap<TrajectoryEnvelope,ArrayList<TrajectoryEnvelope>> refinedWith = new HashMap<TrajectoryEnvelope, ArrayList<TrajectoryEnvelope>>();
	private static final int MINIMUM_SIZE = 5;


	public CopyOfViewSchedulingMetaConstraint(VariableOrderingH varOH,
			ValueOrderingH valOH) {
		super(varOH, valOH);

	}

	private boolean isConflicting(ViewVariable[] viewVariables) {		
		ViewVariable vv1 = (ViewVariable)viewVariables[0];
		ViewVariable vv2 = (ViewVariable)viewVariables[1];
		if (vv1.getTrajectoryEnvelope().getRobotID() == vv2.getTrajectoryEnvelope().getRobotID()) return false;
		if(vv1.getTrajectoryEnvelope().getMarking() != null && vv1.getTrajectoryEnvelope().getMarking().equals("path"))
			System.out.println(vv1.getTrajectoryEnvelope());
		if(vv2.getTrajectoryEnvelope().getMarking() != null && vv2.getTrajectoryEnvelope().getMarking().equals("path"))
			System.out.println(vv2.getTrajectoryEnvelope());

		
		
		GeometricShapeVariable poly1 = vv1.getTrajectoryEnvelope().getEnvelopeVariable();
		GeometricShapeVariable poly2 = vv2.getFoV();
		Geometry shape1 = ((GeometricShapeDomain)poly1.getDomain()).getGeometry();
		Geometry shape2 = ((GeometricShapeDomain)poly2.getDomain()).getGeometry();
		if(shape1.intersects(shape2))
			return true;
		
		poly1 = vv1.getFoV();
		poly2 = vv2.getTrajectoryEnvelope().getEnvelopeVariable();
		shape1 = ((GeometricShapeDomain)poly1.getDomain()).getGeometry();
		shape2 = ((GeometricShapeDomain)poly2.getDomain()).getGeometry();
		if(shape1.intersects(shape2))
			return true;
		logger.finest("Resolving peak "  + Arrays.toString(viewVariables));
		return false;
	}

	@Override
	public void draw(ConstraintNetwork network) {
		// TODO Auto-generated method stub

	}

	@Override
	public ConstraintSolver getGroundSolver() {
		return (((ViewConstraintSolver)this.metaCS.getConstraintSolvers()[0]));
	}

	@Override
	public String toString() {
		return this.getClass().getName(); 
	}

	@Override
	public String getEdgeLabel() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Object clone() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean isEquivalent(Constraint c) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public ConstraintNetwork[] getMetaVariables() {

		ViewConstraintSolver viewSolver= (ViewConstraintSolver)this.getGroundSolver();
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		//logger.finest("Doing binary peak collection with " + activities.size() + " activities...");
		
		Variable[] groundVars = viewSolver.getVariables();
		for (int i = 0; i < groundVars.length-1; i++) {
			for (int j = i+1; j < groundVars.length; j++) {
				ViewVariable a = (ViewVariable)groundVars[i];
				ViewVariable b = (ViewVariable)groundVars[j];
				
				Bounds bi = new Bounds(a.getTrajectoryEnvelope().getTemporalVariable().getEST(), a.getTrajectoryEnvelope().getTemporalVariable().getEET());
				Bounds bj = new Bounds(b.getTrajectoryEnvelope().getTemporalVariable().getEST(), b.getTrajectoryEnvelope().getTemporalVariable().getEET());
				if (bi.intersectStrict(bj) != null && isConflicting(new ViewVariable[] {a, b})) {
					ConstraintNetwork cn = new ConstraintNetwork(null);
					cn.addVariable(a);
					cn.addVariable(b);
					ret.add(cn);
				}
			}
		}
		if (!ret.isEmpty()) {
			return ret.toArray(new ConstraintNetwork[ret.size()]);			
		}

		return (new ConstraintNetwork[0]);
	}





	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {
		
		ConstraintNetwork conflict = metaVariable.getConstraintNetwork();
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		ConstraintNetwork resolver1 = new ConstraintNetwork(null);
		ConstraintNetwork resolver2 = new ConstraintNetwork(null);

		ViewVariable vv1 = (ViewVariable)conflict.getVariables()[0];
		ViewVariable vv2 = (ViewVariable)conflict.getVariables()[1];
		
		TrajectoryEnvelope moveOut1TE = creatMoveBaseTrajectoryEnvelope(vv1);	
		AllenIntervalConstraint meetsMoveOut1 =  new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
		meetsMoveOut1.setFrom(vv1.getTrajectoryEnvelope());
		meetsMoveOut1.setTo(moveOut1TE);
		resolver1.addConstraint(meetsMoveOut1);
		resolver2.addConstraint(meetsMoveOut1);
		
		TrajectoryEnvelope moveOut2TE = creatMoveBaseTrajectoryEnvelope(vv2);	
		AllenIntervalConstraint meetsMoveOut2 =  new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
		meetsMoveOut2.setFrom(vv2.getTrajectoryEnvelope());
		meetsMoveOut2.setTo(moveOut2TE);
		resolver1.addConstraint(meetsMoveOut2);
		resolver2.addConstraint(meetsMoveOut2);

		
		refineTrajectoryEnvelopes(moveOut1TE, vv2.getFoV());
		refineTrajectoryEnvelopes(moveOut2TE, vv1.getFoV());
		
		
		AllenIntervalConstraint before01 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		before01.setFrom(moveOut1TE);			
		before01.setTo(vv2.getTrajectoryEnvelope());
		resolver1.addConstraint(before01);

		
		AllenIntervalConstraint before10 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		before10.setFrom(moveOut2TE);			
		before10.setTo(vv1.getTrajectoryEnvelope());
		resolver2.addConstraint(before10);

		
		//this is not neccesary correct, it needs double check which one is conflicting with others
		
		ret.add(resolver2);
		ret.add(resolver1);
		
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}





	private TrajectoryEnvelope creatMoveBaseTrajectoryEnvelope(ViewVariable vv) {
		ViewConstraintSolver viewSolver= (ViewConstraintSolver)this.getGroundSolver();
		TrajectoryEnvelope moveOutTE = (TrajectoryEnvelope)viewSolver.getTrajectoryEnvelopeSolver().createVariable();
		Trajectory moveOutTrajectory = new Trajectory(getTrajectory(vv.getTrajectoryEnvelope().getRobotID()));				
		moveOutTE.setFootprint(vv.getTrajectoryEnvelope().getFootprint());
		moveOutTE.setTrajectory(moveOutTrajectory);
		moveOutTE.setRobotID(vv.getTrajectoryEnvelope().getRobotID());
		return moveOutTE;
	}


	private String getTrajectory(int robotID) {
		if(robotID == 1) 
			return "paths/rid1_task1.path";
		else
			return "paths/rid1_task4.path";
	}





	@Override
	public void markResolvedSub(MetaVariable metaVariable,
			ConstraintNetwork metaValue) {
		// TODO Auto-generated method stub

	}
	
	private ConstraintNetwork refineTrajectoryEnvelopes(TrajectoryEnvelope var1, GeometricShapeVariable geometricShapeVariable) {
		//ViewConstraintSolver viewSolver= ((ViewConstraintSolver)this.getConstraintSolvers()[0]);
		ViewConstraintSolver viewSolver= (ViewConstraintSolver)this.getGroundSolver();
		TrajectoryEnvelopeSolver solver = viewSolver.getTrajectoryEnvelopeSolver();
		ConstraintNetwork toReturn = new ConstraintNetwork(null);
		GeometryFactory gf = new GeometryFactory();
		Geometry se1 = ((GeometricShapeDomain)var1.getEnvelopeVariable().getDomain()).getGeometry();
		Geometry se2 = ((GeometricShapeDomain)geometricShapeVariable.getDomain()).getGeometry();
		Geometry intersectionse1se2 = se1.intersection(se2);
		
		boolean useDefaultEnvelopeChunks = false;
		
		if (!intersectionse1se2.isValid()) {
			intersectionse1se2 = intersectionse1se2.symDifference(intersectionse1se2.getBoundary());
			logger.info("Intersection " + var1 + " with " + geometricShapeVariable + " invalid - fixing");
		}

		if (intersectionse1se2 instanceof MultiPolygon) {
			logger.info("Intersection " + var1 + " with " + geometricShapeVariable + " too complex - skipping");
			useDefaultEnvelopeChunks = true;
			//return toReturn;								
		}
		
		boolean in  = false;
		int countIn = 0;
		for (int i = 0; i < var1.getPathLength(); i++) {
			Coordinate coord = var1.getTrajectory().getPositions()[i];
			Point point = gf.createPoint(coord);
			if (intersectionse1se2.contains(point) && !in) {
				in = true;
				if (++countIn > 1) {
					logger.info("Reference path of " + var1 + " enters intersection with " + geometricShapeVariable + " multiple times - skipping");
					useDefaultEnvelopeChunks = true;
					break;
					//return toReturn;					
				}
			}
			if (!intersectionse1se2.contains(point)) {
				in = false;
			}
		}

		double areaDifference = intersectionse1se2.symDifference(intersectionse1se2.getBoundary()).union(se1).getArea()-se1.getArea();
		if (areaDifference > 0.001) {
			logger.info("Intersection " + var1 + " with " + geometricShapeVariable + " seems corrupt (area increased by " + areaDifference + ") - skipping ");
			useDefaultEnvelopeChunks = true;
			//return toReturn;											
		}

		// IRAN: UNOCMMENT THIS IF YOU HAVE PROBLEMS WITH SCHEDULING
//		if (!intersectionse1se2.coveredBy(se1)) {
//			logger.info("Intersection " + var1 + " with " + var2 + " is corrupted - skipping");
//			return toReturn;											
//		}

//		logger.info("Refining " + var1 + " with " + var2);

		ArrayList<PoseSteering> var1sec1 = new ArrayList<PoseSteering>();
		ArrayList<PoseSteering> var1sec2 = new ArrayList<PoseSteering>();
		ArrayList<PoseSteering> var1sec3 = new ArrayList<PoseSteering>();

		boolean skipSec1 = false;
		boolean skipSec3 = false;

		if (useDefaultEnvelopeChunks) {
			float percentageChunckOne = 0.30f;
			float percentageChunckTwo = 0.40f;
			for (int i = 0; i < var1.getPathLength(); i++) {
				PoseSteering ps = var1.getTrajectory().getPoseSteering()[i];
				if (i < var1.getPathLength()*percentageChunckOne) var1sec1.add(ps);
				else if (i < var1.getPathLength()*(percentageChunckOne+percentageChunckTwo)) var1sec2.add(ps);
				else var1sec3.add(ps);
			}
			logger.info("Using default chunk sizes " + var1sec1.size() + " / " + var1sec2.size() + " / " + var1sec3.size());
		}
		else {	
			for (int i = 0; i < var1.getPathLength(); i++) {
				Coordinate coord = var1.getTrajectory().getPositions()[i];
				PoseSteering ps = var1.getTrajectory().getPoseSteering()[i];
				Point point = gf.createPoint(coord);
				Geometry fp = var1.makeFootprint(ps);
				if (!intersectionse1se2.intersects(fp) && var1sec2.isEmpty()) {
					var1sec1.add(ps);
				}
				else if (intersectionse1se2.intersects(fp)) {
					var1sec2.add(ps);
				}
				else if (!intersectionse1se2.intersects(fp) && !var1sec2.isEmpty()) {
					var1sec3.add(ps);
				}
	//			if (!intersectionse1se2.contains(point) && var1sec2.isEmpty()) {
	//				var1sec1.add(ps);
	//			}
	//			else if (intersectionse1se2.contains(point)) {
	//				var1sec2.add(ps);
	//			}
	//			else if (!intersectionse1se2.contains(point) && !var1sec2.isEmpty()) {
	//				var1sec3.add(ps);
	//			}
			}
				
			//Add to start
			boolean done = false;
			while (!done) {
				try {
					Geometry lastPolySec1 = var1.makeFootprint(var1sec1.get(var1sec1.size()-1));
					if (lastPolySec1.disjoint(se2)) done = true;
					else {
						var1sec2.add(0,var1sec1.get(var1sec1.size()-1));
						var1sec1.remove(var1sec1.size()-1);
						logger.info("Added to start... (1)");
					}
				} catch (IndexOutOfBoundsException e) 
				{ skipSec1 = true; done = true; }
			}
			//If sec1 emptied, remove it
			if (var1sec1.size() < MINIMUM_SIZE) {
				while (var1sec1.size() > 0) {
					var1sec2.add(0,var1sec1.get(var1sec1.size()-1));
					var1sec1.remove(var1sec1.size()-1);
				}
				skipSec1 = true;
			}
	
			//Add to end
			done = false;
			while (!done) {
				try {
					Geometry firstPolySec3 = var1.makeFootprint(var1sec3.get(0));
					if (firstPolySec3.disjoint(se2)) done = true;
					else {
						var1sec2.add(var1sec3.get(0));
						var1sec3.remove(0);
	//					logger.info("Added to end... (1)");
					}
				} catch (IndexOutOfBoundsException e) { skipSec3 = true; done = true; }
			}
			//If sec3 emptied, remove it
			if (var1sec3.size() < MINIMUM_SIZE) {
				while (var1sec3.size() > 0) {
					var1sec2.add(var1sec3.get(0));
					var1sec3.remove(0);
				}
				skipSec3 = true;
			}
			
			if (var1sec2.size() < MINIMUM_SIZE) {
				if (var1sec1.size() > MINIMUM_SIZE) {
					var1sec2.add(0,var1sec1.get(var1sec1.size()-1));
					var1sec1.remove(var1sec1.size()-1);
	//				logger.info("Added to start... (2)");
				}
				else if (var1sec3.size() > MINIMUM_SIZE) {
					var1sec2.add(var1sec3.get(0));
					var1sec3.remove(0);				
	//				logger.info("Added to end... (2)");
				}
			}
	
				if ((skipSec1 && skipSec3) || (!skipSec1 && var1sec1.size() < MINIMUM_SIZE) || (!skipSec3 && var1sec3.size() < MINIMUM_SIZE) || var1sec2.size() < MINIMUM_SIZE) {
					logger.fine("Intersection " + var1 + " with " + geometricShapeVariable + " too small - skipping");
					return toReturn;
				}
		
		}

		var1.setRefinable(false);
		ArrayList<Trajectory> newTrajectories = new ArrayList<Trajectory>();
		ArrayList<TrajectoryEnvelope> newTrajectoryEnvelopes = new ArrayList<TrajectoryEnvelope>();
				
		if (!skipSec1) {
			newTrajectories.add(new Trajectory(var1sec1.toArray(new PoseSteering[var1sec1.size()]),var1.getTrajectory().getDts(0, var1sec1.size())));
			newTrajectories.add(new Trajectory(var1sec2.toArray(new PoseSteering[var1sec2.size()]),var1.getTrajectory().getDts(var1sec1.size(), var1sec1.size()+var1sec2.size())));
			if (!skipSec3) {
				newTrajectories.add(new Trajectory(var1sec3.toArray(new PoseSteering[var1sec3.size()]),var1.getTrajectory().getDts(var1sec1.size()+var1sec2.size(),var1.getTrajectory().getPoseSteering().length)));
			}
		}
		else {
			newTrajectories.add(new Trajectory(var1sec2.toArray(new PoseSteering[var1sec2.size()]),var1.getTrajectory().getDts(0, var1sec2.size())));
			if (!skipSec3) {
				newTrajectories.add(new Trajectory(var1sec3.toArray(new PoseSteering[var1sec3.size()]),var1.getTrajectory().getDts(var1sec2.size(),var1.getTrajectory().getPoseSteering().length)));
			}			
		}

		Variable[] newVars = solver.createVariables(newTrajectories.size());
		for (int i = 0; i < newVars.length; i++) {
			TrajectoryEnvelope te = (TrajectoryEnvelope)newVars[i];
			//te.setFootprint(var1.getWidth(), var1.getLength(), var1.getDeltaW(), var1.getDeltaL());
			te.setFootprint(var1.getFootprint());
			//Only for second!
//			if ((!skipSec1 && i == 1) || (skipSec1 && i == 0)) {
//				te.setRefinable(false);
//				refinedWith.get(geometricShapeVariable).add(te);
//			}
//			System.out.println("doing i = " + i + " skipsec1: " + skipSec1 + " skipsec3: " + skipSec3);
			te.setTrajectory(newTrajectories.get(i));
			te.setSuperEnvelope(var1);
			te.setRobotID(var1.getRobotID());
			var1.addSubEnvelope(te);
			newTrajectoryEnvelopes.add(te);			
		}

		AllenIntervalConstraint starts = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Starts);
		starts.setFrom(newTrajectoryEnvelopes.get(0));
		starts.setTo(var1);
		toReturn.addConstraint(starts);

		AllenIntervalConstraint finishes = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Finishes);
		finishes.setFrom(newTrajectoryEnvelopes.get(newTrajectoryEnvelopes.size()-1));
		finishes.setTo(var1);
		toReturn.addConstraint(finishes);

		double minTTT12 = 0.0;
		
		if (!skipSec1) minTTT12 = var1.getTrajectory().getDTs()[var1sec1.size()];
		else minTTT12 = var1.getTrajectory().getDTs()[var1sec2.size()];
		long minTimeToTransition12 = (long)(TrajectoryEnvelope.RESOLUTION*minTTT12);
		AllenIntervalConstraint before1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before, new Bounds(minTimeToTransition12,minTimeToTransition12));
		before1.setFrom(newTrajectoryEnvelopes.get(0));
		before1.setTo(newTrajectoryEnvelopes.get(1));
		toReturn.addConstraint(before1);
	
		if (newTrajectoryEnvelopes.size() > 2) {
			double minTTT23 = var1.getTrajectory().getDTs()[var1sec1.size()+var1sec2.size()];
			long minTimeToTransition23 = (long)(TrajectoryEnvelope.RESOLUTION*minTTT23);
			AllenIntervalConstraint before2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before, new Bounds(minTimeToTransition23,minTimeToTransition23));
			before2.setFrom(newTrajectoryEnvelopes.get(1));
			before2.setTo(newTrajectoryEnvelopes.get(2));
			toReturn.addConstraint(before2);
		}

//		System.out.println("var1sec1 (" + skipSec1 + "): " + var1sec1);
//		System.out.println("var1sec2: " + var1sec2);
//		System.out.println("var1sec3 (" + skipSec3 + "): " + var1sec3);
//		System.out.println("DTs of var1sec2: " + Arrays.toString(var1.getTrajectory().getDts( var1sec2.size(),var1.getTrajectory().getDTs().length-1 )));
		solver.addConstraints(toReturn.getConstraints());
		
		return toReturn;
	}



}
