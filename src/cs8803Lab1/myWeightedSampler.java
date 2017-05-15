package cs8803Lab1;

import java.util.concurrent.Callable;
import java.util.concurrent.ThreadLocalRandom;

import processing.core.PApplet;

public class myWeightedSampler implements Callable<Boolean> {//extends Thread {
	public Lab1Localization p;
	public myLocalizer lcl;
	public final float invSqrt2Pi;
	public myLogData log;
	
	public float sigHit;						//set as .1 raycast dist?
	public myMap m;	
	public int dataPointIdx, stIdx, endIdx, numSamps;
	mySample[] XbarAra;	
	public myWeightedSampler(Lab1Localization _p, myLocalizer _l, myLogData _log, myMap _m, mySample[] _XbarAra, int _dataPointIdx, int _stIdx, int _numSamps) {
		p=_p; log=_log; m=_m; lcl = _l;
		this.invSqrt2Pi = Lab1Localization.invSqrt2Pi;
		dataPointIdx = _dataPointIdx;	//index in datapoint array for sequence of odo/LRF readings - NOTE : all data points now have range finder readings too (odo has most recent)
		stIdx = _stIdx;					//start index in proposal dist array
		numSamps = _numSamps;			//# of samples to get from prop array to calculate in this thread
		endIdx = stIdx + numSamps;
		XbarAra = _XbarAra;
		sigHit = 1;	//just initial value
	}

	public float calcHitProb(float ztk_ztkStar, float sigHit){
		float zOvSig = (ztk_ztkStar/sigHit);	
		if(sigHit==0){return 0;}
		return (float)(Lab1Localization.invSqrt2Pi/(sigHit) *Math.exp(-.5f*(zOvSig*zOvSig))); 	
	}
	
	//lamShort * e^-lamshort*ztk / 1 - e^-lamshort*ztkstar
	public float calcShortProb(float ztk, float ztkstar){
		if(ztkstar==0)return 0;
		float res = (float)(myLocalizer.lamshort * Math.exp(-myLocalizer.lamshort * ztk)/(1 - Math.exp(-myLocalizer.lamshort * ztkstar))); 
		return res;}
	
	public float calcSingleObsProb(float zt_k, float ztk_star, float sigHit, float lclMax){
		float pzhit,pzshort,pzmax,pzrand, p_res;
		pzhit =  myLocalizer.zhit * ((zt_k <= 0)||(zt_k>lclMax) ?  0 :  (calcHitProb((zt_k-ztk_star),sigHit)));
		pzshort = myLocalizer.zshort * ((zt_k <= 0)||(zt_k>ztk_star) ?  0 :  (calcShortProb(zt_k,ztk_star)));
		pzmax = myLocalizer.zmax * (zt_k>= .99* lclMax ? 1 : 0);
		pzrand = myLocalizer.zrand * ((zt_k <= 0)||(zt_k>=lclMax) ?  0 :  lcl.invZMax);
		p_res = pzhit + pzshort + pzmax + pzrand;
		if(Float.isNaN(p_res)){
			p.outStr2Scr("NaN result in brf function at specific calc : ztk : " +zt_k+" sample ztstar data point : " +ztk_star);
		}

		return p_res;	
	}
	//calculate range finder model p(z|xt,m) - sample holds relevant ray casts from map
	public float brf_model(myDataPoint d, mySample xt){
		float q1 = 0, //0 if log prob, 1 if prob
			  q2 = 1,
				p_res, ztkStar;
		float[] zt_k = d.lrngCells;
		
		int idx = 0;//xt.scanDist.length-1;
		//int idx = xt.scanDist.length-1;
		int numIncr = zt_k.length/xt.scanDist.length;
		
		for(int k = 0; k<zt_k.length; k+=numIncr){
			ztkStar = xt.scanDist[idx++];
			//ztkStar = xt.scanDist[idx--];
			if(ztkStar==0){continue;}							//raycast will be 0 only if in a wall
			sigHit = myLocalizer.sigScale*ztkStar;				//scale by distance of raycast?  implies it gets bigger/smaller based on distance.
			p_res = calcSingleObsProb(zt_k[k], ztkStar, sigHit, lcl.zMax);
//			float res1 = (float)Math.log(1 + (p_res/(1-p_res)));
//			if(Float.isNaN(res1)){
//				p.outStr2Scr("NaN result in brf function at specific calc : " +k+" data point : " +d +" pres " + p_res);
//			}
			q1+=p_res;
			//q1+=(p_res==0 ? 0 : (float)Math.log(1 + (p_res/(1-p_res))));			//log odds prob
			//q2*=(float)Math.pow(p_res,myLocalizer.measAlpha);
		}		
		float res = (float)Math.pow(q1,myLocalizer.measAlpha);// (q1==0? q2 : q1);
		if(Float.isNaN(res)){
		//	p.outStr2Scr("NaN result in brf function : data point : " +d);
		}
		return res;//(q1==0? q2 : q1);
	}//brf_model
		
	//normal distribution probability
	public float prob(float a, float bsq){return (invSqrt2Pi/PApplet.sqrt(bsq)) * PApplet.exp( -.5f* a*a/bsq);}
	//normal distribution sampling - from equation in the book
	public float sampleDataSq(float bsq){
		double res = 0, b = Math.sqrt(bsq);
		for(int i =0;i<12;++i){	res += ThreadLocalRandom.current().nextDouble(-b,b);}	res *= .5;
		return (float)res;		
	}//	sampleData	
	//normal distribution sampling - from equation in the book
	public float sampleData(float b){
		double res = 0;
		for(int i =0;i<12;++i){	res += ThreadLocalRandom.current().nextDouble(-1,1);}	
		res *= b/6.0;
		return (float)res;		
	}//	sampleData		
	/**
	 * sample motion model for odometry data - same config as motion model for vectors
	 * @param ut  pair of poses from odo data : idx0 == xbar_t_1=(xbar,ybar,thetbar), idx1 == xbar_t=(xbartick,ybartick,thetbartick)
	 * @param xt_1 initial pose 
	 * @return gives a random xt state based on odo distribution described by ut and xt_1
	 */
	//NOTE :X AND Y ARE OFF BETWEEN SAMPLE AND BOT DATA
	public mySample odoSampleMoModelWithoutMap( myPoint[] ut, mySample xt_1){
		//float negMult = -1.0f;
		float diffYbar = ut[1].y-ut[0].y, 
				diffXbar = ut[1].x-ut[0].x, 
				diffThetBar = ut[1].z-ut[0].z,
				delRot1 = PApplet.atan2(diffYbar, diffXbar)-(ut[0].z),
//				diffThetBar = negMult*(ut[1].z-ut[0].z),
//				delRot1 = PApplet.atan2(diffYbar, diffXbar)-(negMult*ut[0].z),
				delTransSq = (diffXbar*diffXbar)+(diffYbar*diffYbar)				
				;	
		
		myVector del = new myVector(delRot1,PApplet.sqrt(delTransSq),diffThetBar-delRot1);	//x==delrot1, y==deltrans, z==delrot2 		
		//float dRot1Sq = delRot1*delRot1, dRot2Sq = del.z*del.z;
//why sq terms?				
//		myVector delHat = new myVector(del.x - sampleData(myLocalizer.alpha1*(dRot1Sq)+myLocalizer.alpha2*(delTransSq)), 	//delHatRot1 = delRot1 - sample(alpha1*delrot1*delrot1 + alpha2*deltrans*delTrans
//				del.y - sampleData(myLocalizer.alpha3*(delTransSq)+myLocalizer.alpha4*(dRot1Sq)+myLocalizer.alpha4*(dRot2Sq)), //delhattrans
//				del.z - sampleData(myLocalizer.alpha1*(dRot2Sq)+myLocalizer.alpha2*(delTransSq)));							//delhatrot2
		float alphaDelTrans = myLocalizer.alpha2*(del.y);
		myVector delHat = new myVector(del.x - sampleData(myLocalizer.alpha1*(del.x)+alphaDelTrans), 	//delHatRot1 = delRot1 - sample(alpha1*delrot1*delrot1 + alpha2*deltrans*delTrans
				del.y - sampleData(myLocalizer.alpha3*(del.y)+myLocalizer.alpha4*(del.x + del.z)), //delhattrans
				del.z - sampleData(myLocalizer.alpha1*(del.z)+alphaDelTrans));							//delhatrot2

		mySample pv = new mySample(p,m,new myVector(xt_1.d.x + (float)(delHat.y* Math.cos(xt_1.d.z + delHat.x)),								//xtick
													xt_1.d.y + (float)(delHat.y* Math.sin(xt_1.d.z + delHat.x)),															//ytick
													xt_1.d.z + delHat.x + delHat.z)																					//thettick						
						,1.0f/lcl.numSamples);
		return pv;
	}

	/**
	 * sample motion model for odometry data WITH A MAP - same config as motion model for vectors
	 * @param ut  pair of poses from odo data : idx0 == xbar_t_1=(xbar,ybar,thetbar), idx1 == xbar_t=(xbartick,ybartick,thetbartick)
	 * @param xt_1 initial pose
	 * @return gives a random xt state based on distribution described by ut and xt_1
	 */
	public mySample odoSampleMoModelWithMap(myPoint[] ut, mySample xt_1){//modify to be samples
		mySample pv; int iters = 0;
		do{
			pv = odoSampleMoModelWithoutMap(ut, xt_1);
			
			int col = (int)(pv.d.x), row = (int)(pv.d.y);
			pv.w = ((col<0) || (col>m.mapWidth) || (row<0) || (row>m.mapHeight)) ? 0 : m.occupancyMap[row][col];		//col and row
			iters++;
//			if(iters >= 20){
//				System.out.println("\ttoo long :"+iters+"|@ x=col:"+col+" y=row:"+row);
//			}
		} while ((pv.w <=0) && (iters < 20));
		if(pv.w != 0 ){ pv.setLRFCastsAtOrient();}
		return pv;
	}//odoSampleMoModelWithMap
	
	public void run(){
		mySample tmp;
		float w;
		//p.outStr2Scr("start thread for : "+stIdx+ " to "+endIdx);
		for(int i =stIdx; i<endIdx; ++i){
			//call motion model for each sample
			tmp = odoSampleMoModelWithMap(log.dataPairs[dataPointIdx],lcl.X[i]);
			if(tmp.w<0){
				XbarAra[i] = null;				
			} else {
				XbarAra[i] = tmp;			
				//call measurement model for each measurement sample and set sample's prob
				if(log.data[dataPointIdx].type==1){
					w = brf_model(log.data[dataPointIdx], XbarAra[i]);
					XbarAra[i].w = w;			
				}
				else{
					XbarAra[i].w /= lcl.numSamples;
				}
			}
		}
		//p.outStr2Scr("done thread for : "+stIdx+ " to "+endIdx);
	}
	
	public Boolean call() throws Exception{
		run();
		return true;		
	}
	
//	//unused functions
//	
//	/**
//	 * motion model for odometry reading - vector holds x,y,theta in x,y,z
//	 * @param xt  hypothesised final pose
//	 * @param ut  pair of poses from odo data : idx0 == xbar_t_1=(xbar,ybar,thetbar), idx1 == xbar_t=(xbartick,ybartick,thetbartick)
//	 * @param xt_1 initial pose
//	 * @return probability of final pose given ut and xt_1
//	 */
//	public float odoMoModelWithoutMap(myVector xt, myVector[] ut, myVector xt_1){
//		float diffYbar = ut[1].y-ut[0].y, diffXbar = ut[1].x-ut[0].x,diffThetBar = ut[1].z-ut[0].z,
//				delRot1 = PApplet.atan2(diffYbar, diffXbar)-ut[0].z,			//atan2(ybartick-ybar,xbartick-xbar)-thetbar
//				diffY = xt.y-xt_1.y, diffX = xt.x-xt_1.x, diffThet =xt.z-xt_1.z,
//				delHatRot1 = PApplet.atan2(diffY,diffX)-xt_1.z;		
//		
//		myVector del = new myVector(delRot1,PApplet.sqrt((diffXbar*diffXbar)+(diffYbar*diffYbar)),diffThetBar-delRot1),	//x==delrot1, y==deltrans, z==delrot2  
//				delHat = new myVector(delHatRot1, PApplet.sqrt((diffX*diffX)+(diffY*diffY)), diffThet-delHatRot1);
//		
//		float dhRot1Sq = delHat.x*delHat.x, dhTransSq = delHat.y*delHat.y, dhRot2Sq = delHat.z*delHat.z;
//				
//		myVector pv = new myVector(prob((del.x-delHat.x), (alpha[0]*(dhRot1Sq)+alpha[1]*(dhTransSq))),								//p1
//				prob(del.y-delHat.y,   (alpha[2]*(dhTransSq)+alpha[3]*(dhRot1Sq)+alpha[3]*(dhRot2Sq))),				//p2
//				prob(del.z-delHat.z,   (alpha[0]*(dhRot2Sq)+alpha[1]*(dhTransSq)))									//p3						
//		);
//		
//		return pv.x*pv.y*pv.z;
//	}
//
//	
//	/**
//	 * motion model for odometry reading WITH A MAP - vector holds x,y,theta in x,y,z
//	 * @param xt  hypothesised final pose
//	 * @param ut  pair of poses from odo data : idx0 == xbar_t_1=(xbar,ybar,thetbar), idx1 == xbar_t=(xbartick,ybartick,thetbartick)
//	 * @param xt_1 initial pose
//	 * @return probability of final pose given ut and xt_1
//	 */	
//	public float odoMoModelWithMap(myVector xt, myVector[] ut, myVector xt_1){
//		float p_noMap = odoMoModelWithoutMap(xt, ut, xt_1);
//		float p_xtGivenMap = m.getProbOfLocInMap(xt.x, xt.y);///m.mapTotProb; //<-normalizer?
//		return p_noMap * p_xtGivenMap;
//	}//odoMoModelWithMap	
		

}//myWeightedSampler class
