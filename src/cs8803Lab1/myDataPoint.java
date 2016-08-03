package cs8803Lab1;

import java.util.ArrayList;

import processing.core.PApplet;
import processing.core.PConstants;

//class to hold object representing an observation, either odometry or laser rangefinder
//NOTE : odometry readings are actually considered controls for the odometry model
public class myDataPoint {
	public Lab1Localization p;	
	public final int ID;
	
	public final float timeStamp;									//timestamp of observation
	private myPoint odoCoordsRaw, lOdoCoordsRaw;  							//coords of bot, in odometry frame, coords of laser range finder in odometry frame
	public myPoint odoCells, lOdoCells;  							//cell coords of bot, in odometry frame, coords of laser range finder in odometry frame
	public final float thetaOdo, lThetaOdo;							//thetaOdo of bot in odometry frame, thetaOdo of laser range finder in odo frame when coords were taken, where 0 is positive x
	//public float thetaSamp, lThetaSamp;						//theta and laser theta in sample frame , where 0 is positive x, and sweep is ccw
	public final int type;											//0 == odometry, 1 == laser
	private final float[] lreadingsRaw;									//laser range finder readings - offset from center of robot by 25 cm - adjusted as read from file.
	public final float[] lrngCells;									//laser range finder readings in grid space - div 10. - locations can be floats	
	//set every odo cell with the most recent range finder data in cell space - initialize this data to all 0's in case we start with odo data and not lrf data
	public static float[] mostRecentLRngCells = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	public int clr;
	public float maxLen;
	
	/**
	 * range readings of laser in cm.  The 180 readings span
		180 degrees *STARTING FROM THE RIGHT AND GOING LEFT*  Just like angles,
		the laser readings are in counterclockwise order.
	 */	
		
	public myDataPoint(Lab1Localization _p, float[] _pos, int _type, float[] _lsrPos, float[] _lr, float[] _lrg10, float _ts) {
		p = _p;
		ID = Lab1Localization.dp_incrID++;
		timeStamp = _ts;
		odoCoordsRaw = new myPoint(_pos[0],_pos[1],_pos[2]);	
		thetaOdo = _pos[2];					//rotation data is backwards from my orientation
		odoCells = new myPoint(odoCoordsRaw.x/10.0f,odoCoordsRaw.y/10.0f,thetaOdo);
		type = _type;
		maxLen = 0;
		if(type==1){
			lreadingsRaw=new float[_lr.length];
			lrngCells = new float[_lr.length];
			System.arraycopy(_lr, 0, lreadingsRaw, 0, _lr.length);
			System.arraycopy(_lrg10, 0, lrngCells, 0, _lrg10.length);
			maxLen = PApplet.max(lrngCells);
			System.arraycopy(_lrg10, 0, mostRecentLRngCells,0,_lrg10.length);
			lOdoCoordsRaw = new myPoint(_lsrPos[0],_lsrPos[1], _lsrPos[2]);
			lThetaOdo = _lsrPos[2];
			lOdoCells = new myPoint(_lsrPos[0]/10.0f,_lsrPos[1]/10.0f,lThetaOdo);
		} else {
			lreadingsRaw=new float[mostRecentLRngCells.length];
			lrngCells=new float[mostRecentLRngCells.length];
			System.arraycopy(mostRecentLRngCells, 0, lrngCells, 0, mostRecentLRngCells.length);		
			lOdoCoordsRaw = new myPoint(odoCoordsRaw);
			lOdoCells = new myPoint(odoCells);
			lThetaOdo = thetaOdo;
		}
		//rotation is backwards
//		thetaSamp = -thetaOdo -p.HALF_PI; 
//		lThetaSamp = -lThetaOdo-p.HALF_PI;
//		odoCells.z = thetaSamp;
//		lOdoCells.z = lThetaSamp;
	}
	
	public float NormDist(float ztk, float sig_hit){	
		float res = (Lab1Localization.invSqrt2Pi/sig_hit) * PApplet.exp(-.5f*(ztk*ztk)/(sig_hit*sig_hit));
		return res;}
	
	public float getMaxLRFRange(){if(this.type!=1){return -1;}float res = 0;for(int i=0;i<lreadingsRaw.length;++i){res = PApplet.max(lreadingsRaw[i], res);}return res;}
	
	public void drawBotDataLoc(){
		drawOdoLoc();
		drawLRFbeams();		
	}
		
	//draws the span of beams from a particular observation
	public void drawLRFbeams(){
		//if(this.type==0){return;}			//no beams from odo reading
		p.pushMatrix();p.pushStyle();
		p.translate(odoCells.x, odoCells.y,0);
		p.rotate(lThetaOdo,0,0,p.rotZ);					//rotate to perceived orientation -- use -1 as axis for right handed
		p.translate(2.5f, 0,0);						//move to where laser range finder vals are
		p.rotate(-PConstants.PI/2.0f,0,0,p.rotZ);			//reading spans
		p.strokeWeight(1);
		//sweep is blue to red - only show 1 ray every deg per cast 
		for(int i=0; i<this.lrngCells.length; i+=p.degPerCasts){
			float mult = i/(1.0f*lrngCells.length);
			p.stroke(255*mult,0,255-255*mult,255);			
			p.line(0,0,0,lrngCells[i],0,0);
			p.rotate(p.degPerC2Rad,0,0,p.rotZ);
		}		
		p.popStyle();p.popMatrix();		
	}

	//display robot on map 
	public void drawOdoLoc(){
		//int y=yb+306;
		p.pushMatrix();p.pushStyle();
		p.translate(odoCells.x, odoCells.y,0);
			p.rotate(lThetaOdo,0,0,p.rotZ);
			p.strokeWeight(.5f);
			p.stroke(clr);
			p.fill(clr);
			p.box(2.5f, 2, 2);
			p.strokeWeight(2);
			p.line(0, 0, 0, 50,0,0);
		p.popStyle();p.popMatrix();
	}	
	//return array of strings to display on screen instead of in console
	public String[] getInfoStrAra(){
		ArrayList<String> res = new ArrayList<String>();
		res.add("Obs ID :"+ID+" Type : "+ getTypeStr() +" Odometry frame pos : ("+ String.format("%.2f",odoCells.x)+","+ String.format("%.2f",odoCells.y)+","+ String.format("%.2f",odoCells.z)+")  Odo orientation ODO frame : " + thetaOdo + " Odo theta Deg : " + String.format("%.2f",(thetaOdo*p.RAD_TO_DEG)) + " Timestamp : " + String.format("%.2f",timeStamp));
		if(type==1){
			res.add("Laser Range Finder readings");
			res.add("LRF Odometry frame pos : ("+ String.format("%.2f",lOdoCells.x)+","+ String.format("%.2f",lOdoCells.y)+","+ String.format("%.2f",lOdoCells.z)+")  LRF Odo orientation ODO frame : " + lThetaOdo );
			String tmpStr = "";
			for(int i=0;i<lrngCells.length;++i){if((i)%20==0){if(tmpStr!=""){res.add(tmpStr);tmpStr="";}} tmpStr+="|"+i+":"+String.format("%.2f",lrngCells[i]);}
			res.add(tmpStr);
		}		
		return res.toArray(new String[0]);		
	}
	
	public String getTypeStr(){	return(type==0?"Odometry":"Laser R.F.");}	
	public String toString(){
		String res = "Obs ID :"+ID+" Type : "+ getTypeStr()+" Odometry frame pos : ("+ String.format("%.2f",odoCoordsRaw.x)+","+ String.format("%.2f",odoCoordsRaw.y)+","+ String.format("%.2f",odoCoordsRaw.z)+")  Odometry orientation : " + thetaOdo+" Timestamp : "+String.format("%.2f",timeStamp) + "\n";
		if(type==1){
			res+="Laser Range Finder readings : \n";
			res+="\tLRF Odometry frame pos : ("+ String.format("%.2f",lOdoCoordsRaw.x)+","+ String.format("%.2f",lOdoCoordsRaw.y)+","+ String.format("%.2f",lOdoCoordsRaw.z)+")  LRF Odometry orientation : " + lThetaOdo + "\n";
			for(int i=0;i<lreadingsRaw.length;++i){res+=""+i+" : "+lreadingsRaw[i]+((i+1)%20==0 ? "\n":"|");}
		}
		return res;
	}
}//myDataPoint
