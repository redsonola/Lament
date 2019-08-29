//
//  PerceptualMappingScheme.h
//  InteractiveTangoReadFromAndroid
//
//  Created by courtney on 12/1/14.
//
//

#ifndef InteractiveTangoReadFromAndroid_PerceptualMappingScheme_h
#define InteractiveTangoReadFromAndroid_PerceptualMappingScheme_h


#endif


namespace CRCPMotionAnalysis
{
    
//enum SoundUnitType { Background, MainMelody, OrnateDetail }; //this has never been implemented
    
class AllMappingSchemas
{
protected:
    std::vector<MotionAnalysisData *> mData;
    std::vector<double> mWeights;
    
    ~AllMappingSchemas()
    {
        for( int i=0; i<mData.size(); i++ )
        {
            if( mData[i] != NULL )
                delete mData[i];
        }
    };
};
    
//continuous output... um, fill out...CDB
class ContinousOutMappingSchema : public AllMappingSchemas,  public SignalAnalysisEventOutput
{
public:
    ContinousOutMappingSchema(long did)
    {
        _id = did;
    };
protected:
    long _id;
};
    
//how often to update current value of a mapping schema -- different from the datawindows, which average over the time.
//this holds a previous value until an update at the specified time.
class MappingTimeCycle
{
protected:
    double val;
public:
    enum TimeType { PHRASE=0, MEASURE=1, SECONDS=2 };
    MappingTimeCycle::TimeType type;
    double stepSize;
    double timeStepCount;
    bool init;
    
    MappingTimeCycle(TimeType t, double s = 1 )
    {
        type = t;
        timeStepCount =0 ;
        stepSize = s;
        init = false;
    };
    
    bool isInit(){ return init; };
    
    void setValue(float v)
    {
        val = v;
    };
    double getValue()
    {
        return val;
    };
    
    double getTimeStep()
    {
        return stepSize;
    }
    
    MappingTimeCycle::TimeType getTimeType()
    {
        return type;
    };
    
    //so this only updates the value if correct timestep, if not, holds on.
    virtual void update(double v, MappingTimeCycle::TimeType timeStep, float seconds=0)
    {
        if( getTimeType() == timeStep || !init)
        {
            //if discrete
            if(timeStep == MappingTimeCycle::PHRASE || timeStep == MappingTimeCycle::MEASURE )
                timeStepCount++;
            else timeStepCount+=seconds;
            
            if( timeStepCount >= stepSize || !init )
            {
                val = v;
                timeStepCount=0;
                
                init = ( val > 0  ); //valid?

            }
        }
    };
    
};
    
//event level output should change name, since... to add EVENT!
class MappingSchema : public SignalAnalysisEventOutput, public AllMappingSchemas
{
protected:
    BeatTiming *beatTimer;
    std::vector<MappingTimeCycle *> timeCycles;
    std::string _name; //set for debug
    long _ID; //id of mapping schema
    float scale; //scales the mood result
    
public:
    enum MappingSchemaType { EVENT=0, CONTINUOUS=2, ORCHESTRAL=1 };
protected:
    MappingSchemaType _mappingtype;
public:

    
    MappingSchema( BeatTiming *timer )
    {
        beatTimer = timer;
        motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, 0));
        _mappingtype = MappingSchemaType::EVENT;
        scale = 1; //no effect

    };
    
    std::vector<ci::osc::Message> getOSC()
    {
        std::vector<ci::osc::Message> msgs;
        return msgs;
    };
    std::string getName()
    {
        return _name;
    };
    
    void setName(std::string name)
    {
        _name = name;
    }
    
    virtual MappingSchemaType getMappingType() { return _mappingtype;  };
    
    virtual void setScale(float s)
    {
        scale = s;
    };

};
    
    
    
class PerceptualEvent : public MappingSchema
{
protected:
    double curMood; //mood on some scale, 1..5 (FOR NOW)
    double minMood, maxMood;
    DataWindow moods;
    std::vector<MappingTimeCycle *> timeCycles;
    float curSeconds;
    
    bool fakeMode;


public:
    
    double getMinMood()
    {
        return minMood;
    }
    
    double getMaxMood()
    {
        return maxMood;
    }
    
    PerceptualEvent(BeatTiming *timer, double window_size = 15) :  MappingSchema(timer)
    {
        //defaults... 5 second window... to start...
        minMood = 1;
        maxMood = 3;
        moods.setWindowSize(window_size);
        _mappingtype = MappingSchemaType::EVENT;
        fakeMode = false;
    };
    
    ~PerceptualEvent()
    {
        if( motionData[0] != NULL )
        {
            delete motionData[0];
        }
    };
    
    void fakeModeOn()
    {
        fakeMode = true;
    }
    
    //translate a finer granularity mapping to a coarser one with specified cutoffs - cutoffs should be percent  of a whole
    //note assumes step of 1. !
    double getNonLinearScalingbyFiat(std::vector<double> cutoffs, double newmax, double mood=-1, double newmin=1)
    {
        //accepts mood as param so that can just use however they got it (ie from whenever time averaging) -- todo: FIX
        if(mood == -1) mood = curMood;
        
        int i = 0;
        bool found = false;
        while( i < cutoffs.size() && !found)
        {
            found = mood <= cutoffs[i] * getMaxMood();
            i++;
        }

        return i;
    }
    
    void setFakeMood(double m)
    {
        fakeMode = true;
        curMood = m;
 //       std::cout << "the current fake mood  is" << m << "\n";
 //       std::cout << "name: " << getName() << std::endl;
    }
    
    virtual MappingSchemaType getMappingType() { return _mappingtype; };
    
    virtual void setMinMaxMood(double mi, double ma)
    {
        minMood = mi;
        maxMood = ma;

        //WTF MATE DISABLING 1/2/2017w
        ( (MotionAnalysisEvent *)motionData[0])->setMinMax(minMood, maxMood);
        
    };
    
    virtual double getCurMood(double windowsz = 0, double seconds = 0) //returns average over window (confusing, but TODO: refactor
    {
  //      std::cout << "name: " << getName() << " and mood is " << curMood << " and is fake? " << fakeMode << std::endl ;
        
       if(fakeMode) return curMood;
        
        double newMood;
        if( windowsz==0 ) //just means default
        {
//            std::cout << std::round(moods.getA << std::endl ;
            newMood = std::max( std::round(moods.getAvg()), minMood);
        }
        else
        {
             newMood = std::max( std::round(moods.getAverageOverWindow(windowsz, seconds)), minMood );
        }
//        std::cout << "seconds: " << seconds << "    ";
//        std::cout << "windowsz: " << windowsz << "          ";
//        std::cout << "min mood is: " << minMood << "     ";
//        std::cout << "avg mood is: " << moods.getAvg() << "     ";
//        std::cout << "New mood is: " << moods.getAverageOverWindow(windowsz, seconds) << std::endl ;
        
        return (int) std::min(newMood, maxMood) ;
            
    };
    
    virtual int getCurMood(int sampWin) //returns average over window (confusing, but TODO: refactor
    {
        return (int) std::round( moods.getAvgOverSample(sampWin) );
    }
    
    virtual void determineMood() = 0;
    
    virtual void updateMotionData()
    {
        motionData[0]->setValue(curMood);
    }
    
    virtual void update(float seconds = 0)
    {
        if(fakeMode) return;
        
        curSeconds = seconds; 
        determineMood();
        moods.push_back(curMood, seconds);
        moods.update(seconds);
        updateMotionData();
        
    };
    
    //TODO fix for all
    virtual double findMood()
    {
        if(fakeMode) return curMood;
        
//        double equalWeight = ( (double) maxMood )/( (double) mData.size() );
        double mood = 0;
        for(int i=0; i<mWeights.size(); i++)
        {
            mood = mood + (mWeights[i]* ((MotionAnalysisEvent *)mData[i])->scaledValuePolyFit());//*equalWeight; //CHANGED!!! 3/4/2016
        }
        mood = mood * scale;
        return mood;
    };
    
    //add a time cycle to update a value at discrete intervals, and else hold it.
    void addTimeCycle( MappingTimeCycle::TimeType t, double s = 1 )
    {
        timeCycles.push_back( new MappingTimeCycle(t,s) );
    };
    
    //mark a phrase or measure or... seconds... can easily update seconds in update... will code if needed
    void updateTimeCycle(MappingTimeCycle::TimeType ts, double windowsz = 0, double seconds = 0 )
    {
        double moodVal = getCurMood(windowsz, seconds);
        for( int i=0; i<timeCycles.size(); i++ )
        {
            timeCycles[i]->update(moodVal, ts, seconds);
        }
    };
    
    //I'm making the bet that I will do this at other levels than just phrase, etc.
    double getTimeCycleValue(MappingTimeCycle::TimeType tt, double stepSize=1)
    {
        bool found = false;
        int index = 0;
        while( !found && index<timeCycles.size() )
        {
            found = ( timeCycles[index]->getTimeType()==tt ) && ( timeCycles[index]->getTimeStep()==stepSize );
            index++;
        }
        
        if(!found)
        {
            std::cout << "Warning! Time cycle value not found in PerceptualEvent.\n";
            return 0;
        }
        else
        {
            if( timeCycles[index-1]->isInit() )
                return timeCycles[index-1]->getValue();
            else
            {
                std::cout << "Warning! Time cycle value not initialized! Initializing now!\n";
                updateTimeCycle(tt); //TODO: if CHANGED to having different step-sizes for each time cycle, causes error, but msg at least tells you why
                return timeCycles[index-1]->getValue();
            }
        }
    }
    
    void phrase(double windowsz = 0, double seconds = 0) //windowsize > 0 will set the average over the data window, else default
    {
        updateTimeCycle(MappingTimeCycle::TimeType::PHRASE, windowsz, seconds);
    };
    
    void measure(double windowsz = 0, double seconds = 0) //windowsize > 0 will set the average over the data window, else default
    {
        for( int i=0; i<timeCycles.size(); i++ )
        {
            updateTimeCycle(MappingTimeCycle::TimeType::MEASURE, windowsz, seconds);
        }
    };
    
    double getPhraseLevelValue(double stepSize=1)
    {
        return getTimeCycleValue( MappingTimeCycle::TimeType::PHRASE, stepSize );
    };
    
};
    
class OrchestralEvent : public PerceptualEvent
{
public:
    OrchestralEvent(BeatTiming *timer, double window_size = 15) :  PerceptualEvent(timer, window_size)
    {
        _mappingtype = MappingSchemaType::ORCHESTRAL;
    };
    
};
    
//assuming this class separation makes sense, for now.
class PerceptualContinuous : public PerceptualEvent
{
public:
    PerceptualContinuous(BeatTiming *timer, double window_size = 2.5) :  PerceptualEvent(timer, window_size)
    {
        //defaults... 5 second window... to start...
        minMood = 1;
        maxMood = 3;
        moods.setWindowSize(window_size);
        _mappingtype = MappingSchemaType::CONTINUOUS;
    };
    
    //TODO: fix values ! 8/8/2015
    virtual double getCurMood(double windowsz = 0, double seconds = 0) //returns average over window (confusing, but TODO: refactor
    {
        double cMood;
        if( windowsz==0 ) //just means default
            cMood = std::max( moods.getAvg(), minMood);
        else cMood =  std::max( moods.getAverageOverWindow(windowsz, seconds), minMood );
        return std::min(cMood, maxMood);
    };
    

};
    
    
};
