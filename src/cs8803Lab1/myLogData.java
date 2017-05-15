package cs8803Lab1;

import java.util.ArrayList;

import processing.core.PApplet;
import processing.core.PConstants;

public class myLogData {
	public Lab1Localization p;	
	public final int ID;
	public static int incrID = 0;
	public final String name;
	public final int laserOffset = 25;								//laser is offset from center of robot by 25 cm.
	
	public final int numRngRdg = 180;								//# of range readings per laser sweep

	public float maxLRCells;
	public int numDataPts,
				numODOPts,
				numLRFPts;
	
	public myDataPoint[] data,lrfData; 
	public myPoint[][] dataPairs, dataPairsLRF;

	public float zi_star = 0;											//avg range finder data over all range finder readings
	public int botColor;
	
	public myPoint[] ODOPath, LRFPath, LRFLaserPath, DataPath;		//paths implied by readings
	
	public final int ODOReading = 0, LASERReading = 1;
	
	public final float invZMax;
	
	public float avgDist=0;
	public int numDists=0;
	
	public int[] araCntDists;		//count of each distance from 0 to 820 in lrf data (cells)

	public myLogData(Lab1Localization _p, String _name) {
		p=_p;ID=incrID++;name=_name;
		numODOPts=0;numLRFPts = 0;
		
		data = loadObsData(name);
		numDataPts = data.length;
		dataPairs = buildDataPairs(data,data.length);
		ArrayList<myDataPoint> tmpAra = new ArrayList<myDataPoint>();
		for(int i =0; i<numDataPts; ++i){
			if(data[i].type==1){tmpAra.add(data[i]);}			
		}
		lrfData = tmpAra.toArray(new myDataPoint[0]);
		dataPairsLRF = buildDataPairs(lrfData,lrfData.length);
		//int lidx = 0,oidx=0;
		botColor = p.color((int)(Math.random()*155),(int)(Math.random()*155),(int)(Math.random()*155));	
		
		maxLRCells = getMaxLRFDist();
		invZMax = 1.0f/maxLRCells;
		araCntDists  = new int[830];		//enough room for longest distance in all data files
		buildLRFDistMap();
	}
	//build a map of all the laser range finder readings
	public void buildLRFDistMap(){
		for(int i = 0; i<lrfData.length;++i){
			for(int j = 0; j<lrfData[i].lrngCells.length;++j){
				araCntDists[(int)(lrfData[i].lrngCells[j])]++;
				numDists++;
				avgDist +=(int)(lrfData[i].lrngCells[j]);
			}		
		}
		avgDist /=(1.0f*numDists);
	}
	
	//build arrays of 2-element Ut arrays used for motion model (pairs of consecutive odometry readings)
	//where idx 0 is t-1, and idx 1 is t dupe 1st reading to have t-1 entry - need to monitor these datapoints to find deltas to use for calcs
	public myPoint[][] buildDataPairs(myDataPoint[] data, int numPoints){
		myPoint[][] res = new myPoint[numPoints][2];
		res[0] = new myPoint[]{new myPoint(data[0].odoCells), new myPoint(data[0].odoCells)};
		for(int i=1;i<numPoints;++i){
			res[i]=new myPoint[]{new myPoint(data[i-1].odoCells), new myPoint(data[i].odoCells)};}	
		return res;
	}	
	
	public myDataPoint[] loadObsData(String logName){
		String[] fileData = p.loadStrings(logName);
		myDataPoint[] tmpAra = new myDataPoint[fileData.length];
		int obsIdx = 0;
		for(String line : fileData){
			char obsType = line.toUpperCase().charAt(0);
			switch (obsType){
				case 'L' : {tmpAra[obsIdx++] = readLaserObs(line);numLRFPts++; break;}
				case 'O' : {tmpAra[obsIdx++] = readOdoObs(line);numODOPts++; break;}
				default :	{p.outStr2Scr("myLogData::loadObsData : Unrecognized observation type in log :"+obsType); break;}
			}
		}//if(line.toUpperCase().charAt(0)=='L'){	tmpAra[obsIdx] = readLaserObs(line);numLRFPts++;} else {	tmpAra[obsIdx] = readOdoObs(line);numODOPts++;}obsIdx++;}	
		return tmpAra;		
	}//loadObsData
	
	public myDataPoint readOdoObs( String line){
		String[] prsStr = line.split("\\s+");
		return new myDataPoint(p, new float[]{Float.parseFloat(prsStr[1]), Float.parseFloat(prsStr[2]),Float.parseFloat(prsStr[3])}, ODOReading, new float[]{},new float[]{},new float[]{},Float.parseFloat(prsStr[prsStr.length-1]));
	}//readOdoObs
	
	public myDataPoint readLaserObs( String line){
		String[] prsStr = line.split("\\s+");
		float[] readings = new float[numRngRdg];
		float[] rngDiv10 = new float[numRngRdg]; //these denote squares distance from center of bot
		//for(int i=0;i<numRngRdg;++i){	readings[i]=Integer.parseInt(prsStr[i+7]) + laserOffset;rngDiv10[i]=readings[i]/10.0f;}	//can't just add laser offset, need to add 	
		//build offset into reading
		for(int i=0;i<numRngRdg;++i){	
			readings[i] =Float.parseFloat(prsStr[i+7]);
			//need to modify reading based on angle - reading is further since lrf is 25 cm in front of bot
			float angle = (90+i)*PConstants.DEG_TO_RAD;
			float tmpVal = (float)(Math.sqrt(readings[i]*readings[i] + 625 - (50*readings[i] * Math.cos(angle))));//law of cosines - offset readings by 25 cm fwd77
			rngDiv10[i]=tmpVal/10.0f;
		}		
		return new myDataPoint(p, new float[]{Float.parseFloat(prsStr[1]), Float.parseFloat(prsStr[2]),Float.parseFloat(prsStr[3])}, LASERReading,new float[]{Float.parseFloat(prsStr[4]),Float.parseFloat(prsStr[5]),Float.parseFloat(prsStr[6])},readings, rngDiv10,Float.parseFloat(prsStr[prsStr.length-1]));	
	}//readLaserObs	
	
//	public float[] learnIntrinsicParams(){
//		float [] res = new float[]{1,1,1,1,300,100};		//returns zhit, zshort, zmax, zrand, sig_hit, lambda_short - needs to start with estimatees of sig_hit and lambda_short
//		int iters = 0;
//		float sum_ei_hit = 0, sum_ei_short = 0, sum_ei_short_zi = 0, sum_ei_max = 0, sum_ei_rand = 0,sum_ei_hit_zresid=0, e_norm;	
//		float[] tmpNormedHitVals, tmpNormedShortVals;
//		
//		float phit = 1, pshort = 1, pmax = 0, prand = 0;			//to calc product of individual probs
//		do{
//			for(int i=0;i<LRFData.length;++i){//each entry in LRFData is zi
//				phit = 1; pshort = 1; pmax = 0; prand = 0;
//				sum_ei_hit = 0;sum_ei_hit_zresid = 0; sum_ei_short = 0;sum_ei_max = 0; sum_ei_rand = 0;
//				
//				tmpNormedHitVals=LRFData[i].calcHitNormedVals(res[idx_sig_hit]);
//				tmpNormedShortVals = LRFData[i].calcShortNormedVals(res[idx_lam_short]);
//				for(int k=0;k<LRFData[i].lreadings.length; ++k){
//					phit *= ((LRFData[i].lreadings[k] < 0)||(LRFData[i].lreadings[k]>maxLRFRange) ?  1 :  (tmpNormedHitVals[k]));
//					pshort *= ((LRFData[i].lreadings[k] < 0)||(LRFData[i].lreadings[k]>LRFData[i].lMean) ?  1 :  (tmpNormedShortVals[k]));
//					pmax += (LRFData[i].lreadings[k] >= maxLRFRange ?  1 :  0);
//					prand  += ((LRFData[i].lreadings[k] < 0)||(LRFData[i].lreadings[k]>=maxLRFRange) ?  0 : invZMax);					
//				}//for each measurement in this datapoint
//				pmax /= LRFData[i].lreadings.length;
//				
//				e_norm = 1.0f/(phit + pshort + pmax + prand);	//normalizer
//				
//				sum_ei_hit += e_norm*phit;
//				sum_ei_short += e_norm*pshort;
//				float tmp = LRFData[i].lMean - zi_star;			//avg of 1 measurement - avg of all measurements
//				sum_ei_hit_zresid += sum_ei_hit *(tmp*tmp);			//residual of this measurements avg from global measurement avg
//				sum_ei_short_zi += e_norm*pshort*LRFData[i].lMean;
//				sum_ei_max += e_norm*pmax;
//				sum_ei_rand += e_norm*prand;	
//				
//			}//for each datapoint
//			
//			res[this.idx_zhit] = sum_ei_hit/LRFData.length;
//			res[this.idx_zshort] = sum_ei_short/LRFData.length;
//			res[this.idx_zmax] = sum_ei_max/LRFData.length;
//			res[this.idx_zrand] = sum_ei_rand/LRFData.length;			
//			res[this.idx_sig_hit] = PApplet.sqrt((1.0f/sum_ei_hit) * sum_ei_hit_zresid);
//			res[this.idx_lam_short] = sum_ei_short/sum_ei_short_zi;
//			iters++;
//		} while(iters<10);
//		return res;
//	}//learnIntrinsicParams	
	

	//returns max range reading from laser range finder for this log
	public float getMaxLRFDist(){float res = 0;for(int j=0;j<data.length;++j){	res=PApplet.max(res, data[j].getMaxLRFCells());}return res;}	
	//return array of strings to display on screen instead of in console
	public String[] getInfoStrAra(){
		ArrayList<String> res = new ArrayList<String>();
		res.add("Data Log : "+name+" # data pts " + data.length + " # ODO pts : "+this.numODOPts + " # LRF pts : " + this.numLRFPts);
		return res.toArray(new String[0]);		
	}
	
	public String toString(){
		String res = "Data Log : "+name+" # data pts " + data.length + " # ODO pts : "+this.numODOPts + " # LRF pts : " + this.numLRFPts+" Max LR Dist from bot ctr, in cells : " + maxLRCells + "\n";
		//res+="Data pts : \n";
		//for(int i =0;i<data.length;++i){res+=data[i];}		
		return res;		
	}
	
}//class myLogData
