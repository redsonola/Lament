//
//  PerceptualSchemas.h
//  InteractiveTangoReadFromAndroid
//
//  Created by courtney on 2/27/15.
//
//

#ifndef InteractiveTangoReadFromAndroid_PerceptualSchemas_h
#define InteractiveTangoReadFromAndroid_PerceptualSchemas_h

/*
Notes: This is the initial setup, having dichotomous measures. Should move to a perceptual sense of distance.
 
 more/medium/less -- refers to weights
 
 Pointy (stacatto) vs Rounded (legato)	peak at onset	medium
	variance derivative	more
	cadence	less
	lots of sharp boleos, kicks*	more
	more turning, giros, ochos*	more
	faster changes of direction (back), ocho cortado
 
 Busyness vs Spatious/Sparse
	Variance	more
	step rate	more
	low cross-covar (leader, follower, different movements)	less
	diverse types of moves*	medium
 
 Room/Area Size, big/small, cavernous/claustrophobic	cruzadas, ganchos, wraps*	more
	cross-covariance	medium
	step time (cadence?)	medium
	longer time to complete turns* (larger turns -- more steps)	more
 more kicks (backward)*	more
	more tight turns, less ochos	more
	more rocksteps*	medium
 
 Thickness vs. Thin / Light / Solo	cross-covariance	more
 Density (Texture) vs. Sparse/Spatious (related)	diverse steps*	medium
	cadence	medium
	variance	medium
 
 
 ***relative step length would be valuable... increase sampling rate somehow?
 ***maybe can find short vs. long
 
 
*/

namespace CRCPMotionAnalysis {

    
//TODO: refactor for a loading class which creates things based on files
    
//TODO: make weights ratios
    
    
//depreciated class!!!!!!!! NOT USED!!!!!!!
/*
class PointyVsSmoothEvent : public PerceptualEvent
{
public:
    enum Factors{ STEP_NUM=0, WINVARDIFF=1 };
    
    PointyVsSmoothEvent(BeatTiming *timer, MotionAnalysisEvent *stepNum,  MotionAnalysisEvent *winVarDiff ) : PerceptualEvent(timer)
    {
        //create set of factors
        mData.push_back( stepNum );
        mData.push_back( winVarDiff );
        
        //change from 1 to 3 FOR NOW
        minMood = 1;
        maxMood = 3;
        
        //add weights to those factors
        mWeights.push_back(0.5);
        mWeights.push_back(2.5);
        
        _name = "PointyVsSmoothEvent";


    };
    
    virtual void determineMood()
    {
        curMood = std::max(std::round(findMood())+1, (double) minMood);
        curMood = std::min(curMood, maxMood);
    };
    
};
 */

class BusyVsSparseEvent : public PerceptualEvent
{
public:
    enum Factors{ WIN_VAR=0, STEP_NUM=1, CROSS_COVAR=2 }; //only one factor for now
    
    float maxMaxMood;
    float minMinMood;
    float moodSum;
    float moodCount;
    
    
    float maxCrossCoVar;
    float minCrossCoVar;
    float crossCoVarSum;
    float crossCoVarCount;
    
    float maxVar;
    float minVar;
    
    
    BusyVsSparseEvent(BeatTiming *timer, MotionAnalysisEvent *winVar,  MotionAnalysisEvent *stepNum,  MotionAnalysisEvent *crosscovar_avg ) : PerceptualEvent(timer)
    {
        //create set of factors
        mData.push_back( winVar );
        mData.push_back( stepNum );
//        mData.push_back( crosscovar_avg ); //testing 8-5-2016
        
        //change from 1 to 3 FOR NOW
        setMinMaxMood(1, 3);
        
        //add weights to those factors --> changed to equal 3
        mWeights.push_back(0.75);
        mWeights.push_back(0.25);
//        mWeights.push_back(0.5);
        
//        maxMaxMood = 0;
//        minMinMood = 0;
//        moodSum = 0;
//        moodCount = 0;
//        
//        maxCrossCoVar = 0;
//        minCrossCoVar = 0;
//        crossCoVarSum = 0;
//        crossCoVarCount = 0;
        
        //testing 10/12/2015
//        maxVar = 0;
//        minVar = 100000;
        
        _name = "BusyVsSparse";
        
    };
    
    //fixed for busy vs sparse herre -- TODO --- propogate to all.
    virtual double findMoodfixed()
    {
        if(fakeMode) return curMood;
        
        //        double equalWeight = ( (double) maxMood )/( (double) mData.size() );
        double mood = 0;
        for(int i=0; i<mWeights.size(); i++)
        {
            mood = mood + (mWeights[i]*(maxMood-minMood+1)* ((MotionAnalysisEvent *)mData[i])->scaledValuePolyFit());//*equalWeight; //CHANGED!!! 3/4/2016
        }
        mood = mood * scale;
        return mood;
    };
    
    //for stepping peak... b
    virtual void updateMotionData()
    {
        const int WINDOW_FOR_USE_AS_MOTION_DATA = 1.5; //TODO: add class var & constructore --
        motionData[0]->setValue(std::max( std::round(moods.getAverageOverWindow(WINDOW_FOR_USE_AS_MOTION_DATA, curSeconds)), minMood ));
    }
    
    virtual void determineMood()
    {
        //just one weight of factors
        curMood = std::max(std::round(findMood()), (double) minMood);
        curMood = std::min(curMood, (double) maxMood);
        
//        std::cout << getName() << "," << curMood << std::endl ;

        
//        if( !_name.compare("Follower Busy Sparse") )
//        {
//            std::cout << getName() << "," << curMood << "  , Values: ";
//        for( int i=0; i<mData.size(); i++ )
//            std::cout  <<  ((MotionAnalysisEvent *) mData[i] )->scaledValuePolyFit() << ","  ;
//        std::cout << std::endl;
//        }
        
//            for( int i=0; i<mData.size(); i++ )
//                std::cout  << ((MotionAnalysisEvent *) mData[i] )->asFloat() << ","  ;
//            std::cout << std::endl;
//        }

    };
    
    virtual double findMood()
    {
        double  m = findMoodfixed();
//        if( !_name.compare("Leader Busy Sparse") )
//            std::cout << "Leader Busy Sparse - in find mood" << "," << m << "\n";

        return std::min(m, maxMood);
    };

    
};
    
class PointyVsSmoothInstrumentEvent : public OrchestralEvent
{
public:
    enum Factors{ STEP_NUM=0, WINVARDIFF=1, PEAK_SIZE=2 };
    
     PointyVsSmoothInstrumentEvent(BeatTiming *timer, MotionAnalysisEvent *stepNum,  MotionAnalysisEvent *winVarDiff, MotionAnalysisEvent *peakSize) : OrchestralEvent(timer)
     {
         
         //create set of factors
         mData.push_back( stepNum );
         mData.push_back( winVarDiff );
         mData.push_back( peakSize ); //just look at values for now
         
         //change from 1 to 3 FOR NOW
         setMinMaxMood(1, 3);
         
         //add weights to those factors
         mWeights.push_back(0.15*3.0);
         mWeights.push_back(0.55*3.0);
         mWeights.push_back(0.30*3.0);
     
     
         _name = "PointyVsSmoothInstrumentEvent";
     
     };
    
    virtual void determineMood()
    {
        //just one weight of factors
        curMood = std::max(std::round(findMood()), (double) minMood);
        curMood = std::min(curMood, (double) maxMood);
        
        //        if( !_name.compare("Follower Busy Sparse") )
        //        {
        //            std::cout << getName() << "," << curMood << "  , Values: ";
        //        for( int i=0; i<mData.size(); i++ )
        //            std::cout  << ((MotionAnalysisEvent *) mData[i] )->scaledValuePolyFit() << "," ;
        //        std::cout << std::endl;
        //        }
        
    };
    
    virtual double findMood()
    {
        double mood = 0;
        for(int i=0; i<mWeights.size(); i++)
        {
            mood = mood + (mWeights[i]* ((MotionAnalysisEvent *)mData[i])->scaledValuePolyFit()); //CHANGED 2/22/2016 //changed scaled value... -- //TODO: FIX
        }
        
//                    std::cout << getName() << ": " << curMood << "  , Values: ";
//                for( int i=0; i<mData.size(); i++ )
//                    std::cout  << ((MotionAnalysisEvent *) mData[i] )->scaledValuePolyFit() << "," ;
//                std::cout << std::endl;
        mood = mood * scale;
        return mood;
    };
    
};
    
    


    
    
    
//NOTE: THESE ARE CURRENTLY NOT USED IN SETUP
//THIS SHOULD BE SWITCHED TO CONTINUOUS!!!
//how large of a space...
/*
 
	cross-covariance	medium
	step time (cadence?)	medium
	longer time to complete turns* (larger turns -- more steps)	more
 more kicks (backward)*	more
	more tight turns, less ochos	more
	more rocksteps*	medium
 
 7-18-2016
 //what about used a measure of similarity between dancer's movements? 
 partners
 plus different couples 
 ++spatial 
++plus how much their staccato/legato & busy/sparse measure...
++mapped to a reverb
 
 
*/
class RoomSizeAreaEvent : public PerceptualEvent
{
public:
    enum Factors{ STEP_NUM=0, CROSS_COVAR=1 }; //only one factor for now
        
    RoomSizeAreaEvent(BeatTiming *timer, MotionAnalysisEvent *stepNum,  MotionAnalysisEvent *crosscovar_avg ) : PerceptualEvent(timer)
    {
        //create set of factors
        mData.push_back( stepNum ); //LONGER steps, soooo fewer... need a distance measure tho.
        mData.push_back( crosscovar_avg );
        
        //add weights to those factors
        mWeights.push_back(3);
        mWeights.push_back(2);
        
        _name = "RoomSizeAreaEvent";

    };
        
    virtual void determineMood()
    {
        //just one weight of factors
        curMood = std::min(std::round(findMood()), (double) 1);
    };
        
};
    
class OrchThickVsThinEvent : public PerceptualEvent
{
    public:
        enum Factors{  STEP_NUM=0, WIN_VAR=1, CROSS_COVAR=2 }; //only one factor for now
        
        OrchThickVsThinEvent(BeatTiming *timer, MotionAnalysisEvent *stepNum, MotionAnalysisEvent *winVar, MotionAnalysisEvent *crosscovar_avg, float window_size=18 ) : PerceptualEvent(timer, window_size)
        {
            //create set of factors
            mData.push_back( stepNum );
            mData.push_back( winVar );
            mData.push_back( crosscovar_avg );
            
            //change from 2 to 4 FOR NOW
            minMood = 2; // don't allow anything below 2 FOR NOW TODO: change!
            maxMood = 3; //for now
            
            //add weights to those factors
            mWeights.push_back(1);
            mWeights.push_back(1.5);
            mWeights.push_back(3);
            
            _name = "OrchThickVsThin";

            
        };
        
        virtual void determineMood()
        {
            //just one weight of factors
            curMood = std::max(std::round(findMood())+1, (double) minMood);
            curMood = std::min(curMood, maxMood);
//            std::cout << "OrchThickThinCurMood: " << curMood << std::endl;

        };
        
    };
    
    
};

#endif
