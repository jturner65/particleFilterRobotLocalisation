package cs8803Lab1;

import java.util.List;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadLocalRandom;

import processing.core.PApplet;
import processing.core.PConstants;

//class holding particle filter code
public class myLocalizer {
	public Lab1Localization p;
	public final int numSamples = 100;
	public myLogData dataLog;

	public myMap m;

	public final float  invZMax;
	public static float zhit = .6f,
	zshort=.35f,
	zmax=.0025f,
	zrand=.1975f,
	//sig_hit=50,			//set instead to be val * raycast dist?
	lamshort=.0028f;	
	
	public static float sigScale = .08f;			//scaling factor for sigma hit
	public static float measAlpha = 1f;		
	
	//motion model noise params
	public static float alpha1 = .0001f,				//odo model rotational noise - init and final rot
						alpha2 = .000001f,				//odo model trans noise - init and final rot
						alpha3 = .0001f,				//odo model trans noise - translation
						alpha4 = .00001f;				//odo model rotational noise - translation
	public final float zMax;
	
	public myVector[] testBrfResults;
	public float maxBRF_y;		//used for display of test brf results
	
	public myVector[] testSampW;
	public float maxSampW_y;		//used for display of test sample weights
	
	public mySample[] testMoModel;
	
	public mySample[] X; // proposal/prior samples
	public ConcurrentSkipListMap<Float, mySample> proposal;
	
	public List<Future<Boolean>> callSampFutures;
	public List<myWeightedSampler> callSampCalcs;
	public myPoint lastSweepTest, initSweepTest;

	public myLocalizer(Lab1Localization _p, myLogData _dl, myMap _m) {
		p = _p;
		dataLog = _dl;
		zMax = dataLog.maxLRFRange/10.0f;
		invZMax = 1.0f/zMax;
		m = _m;
		initProposal(); // initialize particles
		callSampCalcs = new ArrayList<myWeightedSampler>();
		callSampFutures = new ArrayList<Future<Boolean>>();
		p.outStr2Scr("Test measurement model");
		testBRF();
		testMoMdl();
		p.outStr2Scr("Test measurement model done");
		
		
	}

	// initialize proposal/prior distribution off of base map occupancy probs
	// i.e. set up initial partical spread over occupiable area of grid
	public void initProposal() {
		proposal = new ConcurrentSkipListMap<Float, mySample>();
		X = new mySample[numSamples];
		mySample tmp;
		float[] prob = new float[2];
		for (int i = 0; i < numSamples; ++i) {
			tmp = m.sampleMap(prob, numSamples);
			proposal.put(prob[0], tmp);
			X[i] = tmp; // reference to all samples
		}// build initial proposal based on map occupancy probabilities
		p.outStr2Scr("Built map proposal without orientation");
		setInitSampleOrientation();
		p.outStr2Scr("Set orientation arrays");
	}// initProposal
	
	//set sample ray casts for initial samples
	public void setInitSampleOrientation(){
		Tuple<Integer,Integer> key;
		for (int i = 0; i < X.length; ++i) {
			X[i].setLRFCastsAtOrient();		
		}		
	}
	int count = 0;
		// solve localization for particular index in data log array
	public void MCL(int dataPtidx) {
		
		this.maxSampW_y = -1;
		callSampCalcs.clear();
		//callSampFutures.clear();
		mySample tmp;
		float[] tmpprob = new float[2];
		float totWeight = 0, totNormLogWt = 0;
		int sampleFrameSize = p.max(X.length/50, 10);
		mySample[] XbarAra = new mySample[X.length];
		for (int i = 0; i < X.length; i += sampleFrameSize) {
			int finalLen = (i + sampleFrameSize <= X.length ? sampleFrameSize: X.length % sampleFrameSize);
			// p.outStr2Scr("i :" + i + " final len "+finalLen +" x length "+X.length);			
			callSampCalcs.add(new myWeightedSampler(p, this, dataLog, m, XbarAra, dataPtidx, i, finalLen));
		} // sample and eval each sample
		try {callSampFutures = p.th_exec.invokeAll(callSampCalcs);for(Future<Boolean> f: callSampFutures) { f.get(); }} catch (Exception e) { e.printStackTrace(); }		
		proposal = new ConcurrentSkipListMap<Float, mySample>();
		ConcurrentSkipListMap<Float, mySample> X_bar = new ConcurrentSkipListMap<Float, mySample>();	
		//if(dataLog.data[dataPtidx].type!=1){return;}		//don't resample if variance == 1 (which is the case for loops without lrf measurements
		for (int i = 0; i < XbarAra.length; ++i) {
			if((XbarAra[i]==null) 
	//				|| (XbarAra[i].w < (float)Math.log(1 + (1/(numSamples)/(1-1/(numSamples)))))		//resample if too low probability?
			){
				p.outStr2Scr("Null xbarAra @ i = "+i+  " count :"+count);
				tmp = m.sampleProbMap(tmpprob, numSamples);
				//tmp.w /= numSamples;
				XbarAra[i] = tmp;
				XbarAra[i].setLRFCastsAtOrient();		
				//continue;
			}
			if(XbarAra[i].w <= 0){
				p.outStr2Scr("wt 0  xbarAra @ i ="+i+  " count :"+count);		//should never happen
				continue;
			}
			X_bar.put(totWeight, XbarAra[i]);	
			totWeight += XbarAra[i].w;
			maxSampW_y = p.max(maxSampW_y,XbarAra[i].w);
			//p.outStr2Scr("totW:"+totWeight+" last value added : "+XbarAra[i].w );
		}
		count++;
		//sample from x bar - resampling stage
		//if(dataLog.data[dataPtidx].type==1){
		//p.outStr2Scr((dataLog.data[dataPtidx].type==1 ? "LRF Step : " : "ODO Step : ") + " totW:"+totWeight);
		float totProb = 0;
		while (proposal.size() < numSamples) {
			if(totWeight <=0){
				p.outStr2Scr("negative or 0 total weight : " + totWeight+ " reinit prop");
				initProposal();
				return;
			}
			float prob = (float) ThreadLocalRandom.current().nextDouble(totWeight);
			float lowKey = (null == X_bar.lowerKey(prob) ? X_bar.firstKey() : X_bar.lowerKey(prob));
			float hiKey = (null != X_bar.higherKey(prob) ? X_bar.higherKey(prob) : totWeight);
			mySample w_samploc = X_bar.get(lowKey);
			w_samploc.setTotWeight(totWeight, numSamples);
			proposal.put(totProb, w_samploc);
			totProb += hiKey-lowKey;
		}

		X = proposal.values().toArray(new mySample[0]); // reset sample array
		
		
//		} else {
//			X = X_bar.values().toArray(new mySample[0]); // reset sample array			
//		}
	}// MCL
	
	public float calcHitProb(float ztk_ztkStar, float sigHit){
		float zOvSig = (ztk_ztkStar/sigHit);		
		return (float)(Lab1Localization.invSqrt2Pi/(sigHit) *Math.exp(-.5f*(zOvSig*zOvSig))); 	
	}	

	//lamShort * e^-lamshort*ztk / 1 - e^-lamshort*ztkstar
	public float calcShortProb(float ztk, float ztkstar){
		float res = (float)(lamshort * Math.exp(-lamshort * ztk)/(1 - Math.exp(-lamshort * ztkstar))); 
		return res;}
	
	
	//calculate range finder model p(z|xt,m) - sample holds relevant ray casts from map
	public float brf_model(float d, float zst){
		float   q2 = 1, q1 = 0,
				p_res;
		//float[] zt_k = d.lrngCells;
		float sigHit = this.sigScale * zst;
		float pzhit,pzshort,pzmax,pzrand;
				//precalced raycast dist from sample
		pzhit = zhit * ((d < 0)||(d>zMax) ?  0 :  (calcHitProb(d-zst,sigHit)));
		pzshort = zshort * ((d < 0)||(d>zst) ?  0 :  (calcShortProb(d,zst)));
		pzmax = zmax * (d >= .99f*zMax ? 1 : 0);
		pzrand = zrand * ((d < 0)||(d>zMax) ?  0 :  invZMax);
		p_res = pzhit + pzshort + pzmax + pzrand;
		
		//q1+=(p_res==0 ? 0 : (float)Math.log(1 + (p_res/(1-p_res))));			//log prob
		q1+=p_res;			//log prob
		//q2*=(float)Math.pow(p_res,myLocalizer.measAlpha);
			
		float res = (float)Math.pow(q1, measAlpha);// q1*measAlpha;// (q1==0? q2 : q1);
		if(Float.isNaN(res)){
			p.outStr2Scr("NaN result in localizer brf function : data point : " +d + " p_res " + p_res);
		}
		return res;//(q1==0? q2 : q1);

//		q2*=p_res;
//		return q2;// q1;// (q1==0? q2 : q1);
	}//brf_model
	
	
	//approx normal distribution sampling - from equation in the book
	public float sampleData(float b){
		double res = 0;
		for(int i =0;i<12;++i){	res += ThreadLocalRandom.current().nextDouble(-1,1);}	
		res *= b/6.0;
		return (float)res;		
	}//	sampleData	

	public mySample moModelSample( myPoint[] ut, mySample xt_1){
		float negMult = 1.0f;
		float diffYbar = ut[1].y-ut[0].y, 
				diffXbar = ut[1].x-ut[0].x, 
				diffThetBar = negMult*(ut[1].z-ut[0].z),
//				delRot1 = PApplet.atan2(diffYbar, diffXbar)-(negMult*ut[0].z),
//				diffThetBar = (ut[1].z-ut[0].z),
				delRot1 = PApplet.atan2(diffYbar, diffXbar)-(ut[0].z),
				delTransSq = (diffXbar*diffXbar)+(diffYbar*diffYbar)				
				;	
		myVector del = new myVector(delRot1,PApplet.sqrt(delTransSq),diffThetBar-delRot1);	//x==delrot1, y==deltrans, z==delrot2 		
		myVector delHat = new myVector(del.x - sampleData(myLocalizer.alpha1*(del.x)+myLocalizer.alpha2*(del.y)), 	//delHatRot1 = delRot1 - sample(alpha1*delrot1*delrot1 + alpha2*deltrans*delTrans
				del.y - sampleData(myLocalizer.alpha3*(del.y)+myLocalizer.alpha4*(del.x + del.z)), //delhattrans
				del.z - sampleData(myLocalizer.alpha1*(del.z)+myLocalizer.alpha2*(del.y)));							//delhatrot2

		mySample pv = new mySample(p,m,new myVector(xt_1.d.x + (float)(delHat.y* Math.cos(xt_1.d.z + delHat.x)),								//xtick
													xt_1.d.y + (float)(delHat.y* Math.sin(xt_1.d.z + delHat.x)),															//ytick
													xt_1.d.z + delHat.x + delHat.z)																					//thettick						
						,1.0f/numSamples);
		return pv;
	}	
		
	public void testBRF(){//feed with average distance measurement
		testBrfResults = new myVector[(int)zMax+1];
		maxBRF_y = -1;
		float zlen = 0;
		for(int i = 0; i<testBrfResults.length; ++i){
			zlen = zMax *(1.0f*i/testBrfResults.length);
			testBrfResults[i] = new myVector(i, brf_model(zlen, zMax*.5f),zlen);		
			maxBRF_y = p.max(maxBRF_y, testBrfResults[i].y);
		}		
	}
	
	
	public void sweepBRF(){//feed with average distance measurement
		testBrfResults = new myVector[(int)zMax+1];
		maxBRF_y = -1;
		float zlen = 0;
		for(int i = 0; i<testBrfResults.length; ++i){
			zlen = zMax *(1.0f*i/testBrfResults.length);
			testBrfResults[i] = new myVector(i, brf_model(zlen, ((5*p.drawCount)+10) % zMax),zlen);		
			maxBRF_y = p.max(maxBRF_y, testBrfResults[i].y);
		}		
	}
	public void testMoMdl(){
		int numTestSamps = 100;
		testMoModel = new mySample[numTestSamps];
		mySample test = new mySample(p,m,new myVector(50,50,0),1);
		initSweepTest = new myPoint(10,10,1);
		lastSweepTest = new myPoint(120,210, 1.4f);
		myPoint[] test_ut = new myPoint[]{new myPoint(10,10,1), new myPoint(120,210,1.4f)};
		for(int i =0; i<numTestSamps; ++i){
			testMoModel[i]=moModelSample(test_ut,test);
		}		
	}
	public void sweepMoMdl(){
		int numTestSamps = 100;
		testMoModel = new mySample[numTestSamps];
		mySample test = new mySample(p,m,new myVector(50,50,0),1);
		initSweepTest = new myPoint(10,10,(((5*p.drawCount)+10) % 360)*(p.DEG_TO_RAD));
		lastSweepTest = new myPoint(120,210, 0);
		myPoint[] test_ut = new myPoint[]{initSweepTest, lastSweepTest};
		for(int i =0; i<numTestSamps; ++i){
			testMoModel[i]=moModelSample(test_ut,test);
		}		
	}	
	//draws results of map of distances from log data - useful for determining constants for BRF model
	public void drawMoModelSampleDist(){
		p.pushMatrix();	p.pushStyle();
		p.rotate(p.PI, 0,0,p.rotZ);
		p.translate(-m.mapWidth, -1.5f*m.mapHeight,1);
		//p.scale(-1,1,1);
		p.stroke(50,100,200,255);
		p.strokeWeight(2.0f);
		myPoint sampSt = new myPoint(50,50,0);
		p.pushMatrix();p.pushStyle();
			p.translate(initSweepTest.x, initSweepTest.y, 0);
			p.circle(myPoint.ZEROPT, 5);
			p.rotate(initSweepTest.z,0,0,p.rotZ);
			p.stroke(0,0,0,255);
			p.line(0,0,0,20,0,0);
		p.popStyle();p.popMatrix();
		p.pushMatrix();p.pushStyle();
			p.translate(lastSweepTest.x, lastSweepTest.y, 0);
			p.circle(myPoint.ZEROPT, 10);
			p.rotate(lastSweepTest.z,0,0,p.rotZ);
			p.stroke(0,0,0,255);
			p.line(0,0,0,20,0,0);	
		p.popStyle();p.popMatrix();
		p.line(initSweepTest.x, initSweepTest.y, 0, lastSweepTest.x, lastSweepTest.y, 0);
		p.translate(200,20,0);
		p.pushMatrix();p.pushStyle();
			p.translate(sampSt.x, sampSt.y, sampSt.z);		
			p.strokeWeight(1);
			p.stroke(0,255,0,255);
			p.circle(myPoint.ZEROPT, 10);
			p.rotate(0,0,0,p.rotZ);
			p.stroke(0,0,0,255);
			p.line(0,0,0,20,0,0);
			p.strokeWeight(2.0f);
			for(int i=0;i<testMoModel.length;++i){
				p.pushMatrix();	p.pushStyle();			
				p.translate(testMoModel[i].d.x, testMoModel[i].d.y, 0);
				p.circle(myPoint.ZEROPT, 10);
				p.rotate(testMoModel[i].d.z,0,0,p.rotZ);
				p.stroke(0,0,0,255);
				p.line(0,0,0,20,0,0);	
			//	p.point(testMoModel[i].d.x,testMoModel[i].d.y,0);
				p.popStyle();p.popMatrix();
			}		
			p.popStyle();p.popMatrix();
		p.popStyle();p.popMatrix();
	}	
	
	//draws sample weights, scaled by max value
	public void drawSampWeights(){
		p.pushMatrix();	p.pushStyle();
		p.rotate(p.PI, 0,0,p.rotZ);
		p.translate(0, -1.5f*m.mapHeight,1);
		p.stroke(0,180,100,255);
		p.strokeWeight(1);
		for(int i=0;i<dataLog.araCntDists.length;++i){
			p.pushMatrix();	p.pushStyle();
			//p.translate(-i*2,0);
			p.translate(-i,0);
			p.line(0, 0, 0, 0,50* (float)(Math.log(dataLog.araCntDists[i]+1)),0);
			//p.line(1, 0, 0, 1,100* (float)(Math.log(dataLog.araCntDists[i]+1)),0);

			p.popStyle();p.popMatrix();
		}		
		p.popStyle();p.popMatrix();
	}
	//draws results of map of distances from log data - useful for determining constants for BRF model
	public void drawNumDistsGraph(){
		p.pushMatrix();	p.pushStyle();
		p.rotate(p.PI, 0,0,p.rotZ);
		p.translate(0, -1.5f*m.mapHeight,1);
		p.stroke(0,180,100,255);
		p.strokeWeight(1);
		for(int i=0;i<dataLog.araCntDists.length;++i){
			p.pushMatrix();	p.pushStyle();
			//p.translate(-i*2,0);
			p.translate(-i,0);
			p.line(0, 0, 0, 0,50* (float)(Math.log(dataLog.araCntDists[i]+1)),0);
			//p.line(1, 0, 0, 1,100* (float)(Math.log(dataLog.araCntDists[i]+1)),0);

			p.popStyle();p.popMatrix();
		}		
		p.popStyle();p.popMatrix();
	}
	public void drawMeasurementTestRes(){
		p.pushMatrix();	p.pushStyle();
		p.rotate(p.PI, 0,0,p.rotZ);
		p.translate(testBrfResults.length, -1.5f*m.mapHeight,1);
		p.stroke(222,50,0,255);
		p.strokeWeight(1);
		for(int i=0;i<testBrfResults.length;++i){
			p.pushMatrix();	p.pushStyle();
			p.translate(-testBrfResults[i].x,0);
			p.line(0, 0, 0, 0,(800.0f/maxBRF_y)*testBrfResults[i].y,0);
			p.popStyle();p.popMatrix();
		}		
		p.popStyle();p.popMatrix();
	}
	public void draw(int botIdx) {
		p.pushMatrix();	p.pushStyle();
		if(p.flags[p.sweepBRFCast]){sweepBRF(); sweepMoMdl();}
		drawMeasurementTestRes();
		drawMoModelSampleDist();
		drawNumDistsGraph();
		//p.translate(-m.mapWidth/2.0f, 0,0);//debug only
		m.moveToMapCtr(); // move to where prob map is - white for open, black is blocked
		p.strokeWeight(1);
		if(p.flags[p.debugMode]) {for (int i =0;i<X.length;++i){X[i].draw(i);}} // draw each sample w/name/id
		else {for (int i =0;i<X.length;++i){X[i].draw();}}
		if(p.flags[p.showSampleRCs]){for (int i =0;i<X.length;++i){X[i].drawPreCompRCsbeams();} }//draw pre-comp raycasts
		p.popStyle();	p.popMatrix();
		p.pushMatrix();	p.pushStyle();
		m.translateBotToDemoMapCtr();
		dataLog.data[botIdx].drawBotDataLoc(); // draw sim showing bot moving
		p.popStyle();p.popMatrix();
	}//draw

	public ArrayList<String> getSampleDataAra(){
		ArrayList<String> res = new ArrayList<String>();
		List<String> tmp;
		for(int i =0; i<X.length;++i){
			tmp=X[i].getToStrAra(i);
			res.addAll(tmp);		
		}		
		return res;
	}
}
