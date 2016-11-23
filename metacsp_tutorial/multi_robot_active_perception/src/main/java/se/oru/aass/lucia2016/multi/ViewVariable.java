package se.oru.aass.lucia2016.multi;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Domain;
import org.metacsp.framework.Variable;
import org.metacsp.framework.multi.MultiVariable;
import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.allenInterval.AllenInterval;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.DE9IMRelation;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatial.DE9IM.PolygonalDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.util.AffineTransformation;


/**
 * @author Iran Mansouri
 */
public class ViewVariable extends MultiVariable implements Activity{

	private Polygon FOV = null;
	private Polygon FoVPolygon = null;
	private double infoGain = 0.0;
	
	public ViewVariable(ConstraintSolver cs, int id,
			ConstraintSolver[] internalSolvers, Variable[] internalVars) {
		super(cs, id, internalSolvers, internalVars);		
	}

	
	//set the field of view dimension and set the domain
	public void setFOVCoordinates(Coordinate ... FoVcoords) {		
		GeometryFactory gf = new GeometryFactory();
		Coordinate[] newCoords = new Coordinate[FoVcoords.length+1];
		for (int i = 0; i < FoVcoords.length; i++) {
			newCoords[i] = FoVcoords[i];
		}
		newCoords[newCoords.length-1] = FoVcoords[0];
		FoVPolygon = gf.createPolygon(newCoords);		
		Pose obsPose = ((TrajectoryEnvelope)this.getInternalVariables()[0]).getTrajectory().getPose()[0];  
		AffineTransformation at = new AffineTransformation();
		at.rotate(obsPose.getTheta());
		at.translate(obsPose.getX(), obsPose.getY());
		Geometry rect = at.transform(FoVPolygon);
		PolygonalDomain env = new PolygonalDomain(this,rect.getCoordinates());
		
		this.setDomain(env);
	}
	
	
	public TrajectoryEnvelope getTrajectoryEnvelope(){
		return (TrajectoryEnvelope)this.getInternalVariables()[0];
	}
	
	public GeometricShapeVariable getFoV(){
		return (GeometricShapeVariable)this.getInternalVariables()[1];
	}
	
	public SelectionVariable getSelectionVar(){
		return (SelectionVariable)this.getInternalVariables()[2];
	}

	public RobotVariable getRobotVar(){
		return (RobotVariable)this.getInternalVariables()[3];
	}

	
	@Override
	public int compareTo(Variable o) {
		// TODO Auto-generated method stub
		return 0;
	}


	@Override
	protected Constraint[] createInternalConstraints(Variable[] variables) {
		// TODO Auto-generated method stub
		return null;
	}


	@Override
	public void setDomain(Domain d) {
		this.getInternalVariables()[1].setDomain(d);		
	}


	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return "viewVariable ID " + this.id;
	}


	@Override
	public AllenInterval getTemporalVariable() {
		return this.getTrajectoryEnvelope().getTemporalVariable();
	}


	@Override
	public String[] getSymbols() {
		return this.getTrajectoryEnvelope().getSymbols();
	}


	@Override
	public Variable getVariable() {
		return this;
	}
	
	public void setInfoGain(double infoGain) {
		this.infoGain = infoGain;
	}
	
	public double getInfoGain() {
		return infoGain;
	}

}
