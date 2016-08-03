package cs8803Lab1;

import java.util.ArrayList;
import java.util.TreeMap;

import processing.core.PConstants;

public class mySample {
	public Lab1Localization p;		
	public myVector d;
	public myMap m;
	//public float orient;
	public float[] scanDist;				//sample distances at -pi/2.0f, to pi/2.0f from straight ahead, step pi/10.0f - set these in measurement for zstar
	public float w;				//log_weight
	public final int numSweepCasts;
	public float totWeight;
	public int totNumSamp;
	public float colorMult;	//this is a scaling factor to scale the color of the samples so that lower prob samples are lighter than higher prob ones
	public int[] clr,fclr,dclr,dfclr;
	public mySample(Lab1Localization _p, myMap _m, myVector _d, float _w) {
		p = _p;
		m = _m;
		numSweepCasts = (int)(180/p.degPerCasts);
		d = new myVector(_d);
		//orient = d.z;
		scanDist = new float[numSweepCasts];
		w = _w;
		totWeight=1;			//use to scale sample to show most heavily weighted sample
		colorMult = 1;
		clr = new int[]{100,0,100,255};
		fclr = new int[]{255,0,255,255};
		dclr = new int[]{0,140,0,255};
		dfclr = new int[]{0,255,0,255};
	}
		
	public void set(mySample _s){
		p=_s.p;
		m = _s.m;
		d = new myVector(_s.d);
		//orient = _s.orient;
		scanDist = _s.scanDist;//sample distances 
		w = _s.w;		
	}
	public void setTotWeight (float _tW, int _tNumSamp){
		totWeight = _tW;
		totNumSamp = _tNumSamp;
	}

	//sets orientation of sample
	//mapCasts is array of pre-calculated raycasts at this x,y location
	public void setOrient(float _o){
		d.z=_o;
//		orient=d.z;
	}	
	//checks if sample is close enough to another sample to discard
	public boolean isCloseEnough(mySample other){		return(this.d._SqrDist(other.d) <= .1f);}
	
	
	//return an array of casts at a particular cell spaced appropriately - interpolate if necessary to correspond to 180 degree sweep, with a certain # of casts per 180 sweep
	//because of setting graphics up with stupid left hand system, NEED TO USE -orient
	//raycasts correspond to -pi/2 to pi/2 around orient, stepped every p.degPerCasts, and drawn from pre-calced ray casts, interpolated if necessary 
	public void setLRFCastsAtOrient(){//mapPreScanRayCasts
		Tuple<Integer,Integer> key = new Tuple<Integer,Integer>((int)d.x, (int)d.y);
//		orient = d.z;
		Float[] cellCastAra = m.mapPreScanRayCasts.get(key);
		if(cellCastAra == null){
			p.outStr2Scr("Null cell cast ara for sample : "+ key);
			scanDist = new float[numSweepCasts];
			return;
		}
		int numCasts = cellCastAra.length;												//# of casts pre-calced per cell - total 360 span of z* for cell, use for mod to find idx
		float posOrient = legitAngleRad(d.z);										//remove the possibility of negative angles for mod calc from -pi/2 to pi/2 around orient
		//float posOrientStart = legitAngleRad(posOrient + PConstants.HALF_PI);
		float posOrientStart = legitAngleRad(posOrient - PConstants.HALF_PI);
		float interp = ((posOrient)%p.degPerC2Rad)/p.degPerC2Rad;						//interpolant will give appropriate value for range (from 0 to 1), based on adjacent beams
		
		float[] res = new float[numSweepCasts];								//only want casts for 180 degrees
		int cellIdxOffset = (int)((posOrientStart/p.degPerC2Rad)+numCasts)%numCasts,	  //offset of where to start is first idx below 90 degrees less than passed orientation
			nextCellIdx = (cellIdxOffset+1)%numCasts;
		System.out.print("Pos Orient : "+posOrient+" PosOrientStart : "+ posOrientStart + " cellIdxOffset : " + cellIdxOffset + " interp : "+interp+"\t");
		for(int i=0; i<numSweepCasts; ++i){													//moving halfway around == getting 1/2 of casts
			res[i]=cellCastAra[cellIdxOffset] + interp*(cellCastAra[nextCellIdx]-cellCastAra[cellIdxOffset]);		//get interpolated value corresponding to where orientation lies in range of p.degPerC2Rad
			System.out.print("|" + i + " : "+ cellIdxOffset);
			cellIdxOffset = nextCellIdx;	
			nextCellIdx = (cellIdxOffset+1)%numCasts;
		}
		System.out.println();
		scanDist = res;
	}//getLRFCastsAtOrient
	
	//returns legal positive angle in radians
	public float legitAngleRad(float angle){return ((angle % PConstants.TWO_PI) + PConstants.TWO_PI)%PConstants.TWO_PI;}
	//draw particle sample on map
	public void draw(){
		p.pushMatrix();p.pushStyle();
			p.stroke(clr[0],clr[1],clr[2],clr[3]);
			p.fill(fclr[0],fclr[1],fclr[2],fclr[3]);
			p.translate(d.x,d.y,0);
			p.sphere(1.0f);
			p.rotate(d.z,0,0,p.rotZ);
			p.translate(2, 0);
			p.box(4,1,1);
		p.popStyle();p.popMatrix();		
	}
	//draw particle sample on map with id
	public void draw(int idx){
		p.pushMatrix();p.pushStyle();
			p.stroke(dclr[0],dclr[1],dclr[2],dclr[3]);
			p.fill(dfclr[0],dfclr[1],dfclr[2],dfclr[3]);
			p.translate(d.x,d.y,0);
			p.sphere(1);
			p.pushMatrix();p.pushStyle();
				p.stroke(0,0,0,255);
				p.fill(0,0,0,255);
				p.translate(0,0,3);
				p.text("i="+idx,0,0);
			p.popStyle();p.popMatrix();		
			p.rotate(d.z,0,0,p.rotZ);
			p.translate(2, 0);
			p.box(4,1,1);
		p.popStyle();p.popMatrix();		
	}
	
	//draws the span of beams from a particular observation
	public void drawPreCompRCsbeams(){
		p.pushMatrix();p.pushStyle();
		p.translate(d.x,d.y,0);
		p.rotate((d.z - p.HALF_PI), 0,0,p.rotZ);
		p.strokeWeight(1);
		//sweep is blue to red
		int i =0;
		for(i=0; i<this.scanDist.length; ++i){
			float mult = i/(1.0f*scanDist.length);
			p.stroke(255*mult,0,255-255*mult,255);			
			p.line(0,0,0,scanDist[i],0,0);
			p.rotate(p.degPerC2Rad,0,0,p.rotZ);
		}		
		p.popStyle();p.popMatrix();		
	}//drawPreCompRCsbeams
	
	public ArrayList<String> getToStrAra(int si){
		ArrayList<String> araList = new ArrayList<String>();
		araList.add("Sample " + si + " : x,y,theta :"+d+ " weight :"+ w+ " degrees : "+ (d.z * p.RAD_TO_DEG));
		String res = "";
		for(int i =0;i<scanDist.length;++i){
			res += ""+i+" : "+String.format("%.2f",scanDist[i])+"| ";
			if(i%10==9){	araList.add(res);	res="";	}	
		}	
		araList.add(res);
		return araList;
	}
	
	public String toString(){
		String res = "";
		res += "Sample : x,y,theta :"+d+ " weight :"+ w+ " degrees : "+ (d.z * p.RAD_TO_DEG)+"\n";
		return res;
	}
	
}
