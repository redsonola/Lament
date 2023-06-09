//
//  PeakDetection.h
//  VideoAndOSCLab
//
//  Created by Courtney Brown on 11/1/18, modifying from my previous version for ITM.
//  This class detects peaks in signals & tests whether these peaks meet a specified threshold
//

#ifndef PeakDetection_h
#define PeakDetection_h

namespace CRCPMotionAnalysis {


#define MIN_PEAK_WINDOW 10 //should define in relation to the sample rate
    
//this lets you know whether or not to wait between denoting/reporting peaks
class PeakTimer
{
protected:
    double waitTime; //how long to wait
    double peakTime; //when was the last peak?
    double curTime; //what time is it now?
public:
    PeakTimer()
    {
        waitTime = 0;
        peakTime = 0;
        curTime = 0;
    };
    
    PeakTimer(double w)
    {
        waitTime = w;
    };
    
    void setWaitTime(double wt)
    {
        waitTime = wt;
    };
    
    void peakHappened()
    {
        peakTime = curTime;
    };
    
    void update(double seconds)
    {
        curTime = seconds;
    };
    
    bool readyForNextPeak()
    {
        return ( (peakTime + waitTime) < curTime );
    };
};

//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------

    //the actual ugen for finding peaks
class FindPeaks: public SignalAnalysis
{
protected:
    PeakTimer peakTimer;
    double curPeakHeight;
    MocapDeviceData::DataIndices whichAxisPeak;
    double xThresh, yThresh, zThresh;
    float  waitBetweenPeaks;
    bool combinedPeak; //whether there is a peak in the x y or z signals
    int _id;
    std::string whichBodyPart;
    
    Derivative derivative; //we need this to find the peaks according to matlab func. using
    int newSamples;
    
//    //send a note for each peak (obv. optional)
//    MelodyGenerator *melodyGenerator;
//    MidiNote note;
    
public:
    FindPeaks( float waitBetween, SignalAnalysis *s, int id_, std::string whichBodyPart_="", int bufferSize=12) : SignalAnalysis( s, bufferSize, NULL ), derivative(s, buffersize)
    {
        combinedPeak = false;
        _id = id_;
        whichBodyPart = whichBodyPart_;
        
        //TODO: & note--> this can be changed with a bpm, if that changes -- I have code for this, if relevant
        waitBetweenPeaks = waitBetween;
        peakTimer.setWaitTime( waitBetweenPeaks ); //eg, for the space of an 8th or 16th note... depends if you are using this for step detection
        
        //these values don't mean anything unless comnbinedPeak is true
        curPeakHeight = 0; //current peak, of all axes
        whichAxisPeak = MocapDeviceData::DataIndices::ACCELX; //just a default
        newSamples = 0;
        
//        melodyGenerator = NULL;
//        note = NULL;
    };
    
//    void setMelodyGenerator(MelodyGenerator *melodyGenerator_)
//    {
//        melodyGenerator = melodyGenerator_;
//    }
    
    virtual int getNewSampleCount()
    {
        return newSamples;
    };
    
    bool getCombinedPeak()
    {
        return combinedPeak;
    };
    
    void setThreshes(double x, double y, double z)
    {
        xThresh = x;
        yThresh = y;
        zThresh = z;
    };
    
    //this is normalized
    double getCurrentPeak()
    {
        return curPeakHeight;
    };
    
    //finds a peak in 1d buffered signal
    void findpeaks(double input[], std::vector<double> &peaks, std::vector<int> &indices, int sz, int start )
    {
        //https://octave.sourceforge.io/signal/function/findpeaks.html --> documentation
        
        /* this algorithm is lifted straight from matlab/octave copied here:
         y = y(:)';
         
         switch length(y)
         case 0
         ind = [];
         case 1
         ind = 1;
         otherwise
         dy = diff(y);
         not_plateau_ind = find(dy~=0);
         ind = find( ([dy(not_plateau_ind) 0]<0) & ([0 dy(not_plateau_ind)]>0) );
         ind = not_plateau_ind(ind);
         if y(1)>y(2)
         ind = [1 ind];
         end
         if y(end-1)<y(end)
         ind = [ind length(y)];
         end
         end
         
         if nargout > 1
         peaks = y(ind);
         end
         
         the important thing here is to test the cases at the edge, since this works with buffered sound instead of with complete vector like matlab
         that is how I have altered the function other than xlating into C++
         */
        
        //find non-zero
        std::vector<int> not_plateau_ind;
        for(int i=start; i<sz; i++)
        {
            if( input[i]!=0 ) not_plateau_ind.push_back(i);
        }
        
        if(not_plateau_ind.empty()) return;
        for( int i=1; i<not_plateau_ind.size()-1; i++ )
        {
            if( ( input[not_plateau_ind[i+1]] < 0 )  &  ( input[not_plateau_ind[i]] > 0 )  )
            {
                indices.push_back( not_plateau_ind[i]  );
            }
        }
        
        /* expanding the buffer input to this function takes care of edge cases (rather than the matlab version) */
        for( int i=0; i<indices.size(); i++)
            peaks.push_back(input[indices[i]]);
        
    }
    
    //find the peak for each axis.
    bool findIC(int dataIndex, double &curpeak, double thresh=0.75 )
    {
        std::vector<double> inputX;
        double input[data1.size()];
        std::vector<double> xmin;
        std::vector<int> xminIndex;
    
        //this mess is so I don't have to change the datatypes, hack hack hack

        for(int i=0; i<data1.size(); i++)
        {
            input[i] = data1[i]->getData(dataIndex);
        }

        int windowSize;
        
//        if(data1[0]->getDeviceType()==MocapDeviceData::MocapDevice::IPHONE) //assume all data comes from the same device
        windowSize = data1.size() - getNewSampleCount(); //really where we start in the data buffer
        
//        std::cout << "start: " << windowSize
//        else
//        {
//            windowSize = data1.size() - 210; //really where we start in the data buffer
//        }

        
        bool FoundPeakX = false; //have not found the peak yet
        //    void findpeaks(double input[], std::vector<double> &peaks, std::vector<int> &indices, int sz, int start )

        findpeaks( input, xmin, xminIndex, data1.size(), windowSize ); //send data to the re-written matlab function
        
        //makes sure the peaks found are greater than the threshhold
        //also finds the curPeakHeight
        curpeak = 0.0;

//                std::cout << "xmin[i]: "; //NOTE: print out these values to check for what your thresholds should be
        for( int i=0; i<xminIndex.size(); i++ )
        {
//            std::cout << xmin[i] << " , index: "  << xminIndex[i] << " , " ;
            if(!FoundPeakX) FoundPeakX = (xminIndex[i] >= windowSize+4) && (xmin[i] > thresh );
            if((xminIndex[i] >= windowSize+4))
                curpeak = std::max( curpeak, xmin[i]);
        }
        
//        std::cout << "FoundPeakX: "  << FoundPeakX << " thresh: " << thresh;

//        std::cout << std::endl;
        
        

        return FoundPeakX;
    };
    
    
    MocapDeviceData::DataIndices whichAxisWasPeak()
    {
        return whichAxisPeak;
    };
    
    void increaseXThresh(float amt)
    {
        xThresh += amt;
    }
    void increaseYThresh(float amt)
    {
        yThresh += amt;
    }
    void increaseZThresh(float amt)
    {
        zThresh += amt;
    }
    
    void printThreshes()
    {
        std::cout << "Peak Threshes: " << xThresh << "," << yThresh << "," << zThresh << std::endl;
    }
    
    //returns peak height normalized from 0 to 1. 0 would be equal to the threshhold value and 1 is the max value of the sensor
    double findNormalizedPeakHeight(double x, double y, double z)
    {
        //for now assume this is 1 since we are now scaling all to 0-1
        double scaledValueMaxX = 1;
        double scaledValueMaxY = 1;
        double scaledValueMaxZ = 1;
    
        //scale 0-1 for like to like comparisions...
        double normX =  ( x - xThresh) / ( scaledValueMaxX - xThresh );
        double normY =  ( y - yThresh) / ( scaledValueMaxY - yThresh );
        double normZ =  ( z - zThresh) / ( scaledValueMaxZ - zThresh );
        
        //find out which axis had the highest peak & return that
        double p = normX;
        whichAxisPeak = MocapDeviceData::DataIndices::ACCELX;
        if( normX < normY )
        {
            if( normY < normZ )
            {
                whichAxisPeak = MocapDeviceData::DataIndices::ACCELZ;
                p = normZ;
            }
            else
            {
                whichAxisPeak = MocapDeviceData::DataIndices::ACCELY;
                p = normY;
            }
        }
        else if( normX < normZ  )
        {
            whichAxisPeak = whichAxisPeak = MocapDeviceData::DataIndices::ACCELZ;
            p = normZ;
        }
        
        if(isnan(p)) p =0;
        return p;
    };
    
    //finds if there was a peak & how high it was & which axis
    virtual void update(float seconds)
    {
        combinedPeak = false;

        SignalAnalysis::update(seconds);
        derivative.update();
        data1 =  derivative.getBuffer();

        
        if( data1.size() < MIN_PEAK_WINDOW ) return ; //this is only useful if there is enough of a buffer
        
        newSamples += derivative.getNewSampleCount();
        
        if( getNewSampleCount() <= 10 )
        {
            
            return ; //don't bother if no new samples
            
        }
        
        //debug.
//        if( !whichBodyPart.compare("LeftHand") )
//        {
//            for(int i=data1.size()-derivative.getNewSampleCount(); i<data1.size(); i++)
//            {
//                std::cout << whichBodyPart << getNewSampleCount() << std::endl;
//            }
//        }
        
        peakTimer.update(seconds);
        curPeakHeight = NO_DATA;
        
        double peakx=0, peaky=0, peakz=0;
        
        combinedPeak = ( findIC(MocapDeviceData::DataIndices::ACCELX, peakx, xThresh) || findIC(MocapDeviceData::DataIndices::ACCELY, peaky, yThresh) || findIC(MocapDeviceData::DataIndices::ACCELZ, peakz, zThresh) );
        
        //we only count peaks that occur far enough apart to be considered a separate event
        combinedPeak = ( combinedPeak ) && ( peakTimer.readyForNextPeak() );
        
//        if(!peakTimer.readyForNextPeak())
//            std::cout << "not ready!!\n";

        if(combinedPeak) //reset peak timer
            peakTimer.peakHappened();
        
        curPeakHeight = findNormalizedPeakHeight(peakx, peaky, peakz); //find how big the peak was.
        
//        if(combinedPeak && melodyGenerator != NULL)
//        {
//            std::vector<MidiNote> notes = melodyGenerator->getCurNotes();
//            if(notes.size()>0)
//                note = notes[0];
//        }
        
        newSamples = 0; //reset
    };
    
    virtual std::vector<ci::osc::Message> getOSC()
    {
        
        std::vector<ci::osc::Message> msgs;
        
        //TODO: add OSC messages here -- eg. if peak is true create and send a message
        
        //this tests the peak -- the count is to differentiate it from the last peak
        int isPeak = getCombinedPeak();
        ci::osc::Message peakMessage;
        peakMessage.setAddress(NOTCH_PEAK);
        peakMessage.append(whichBodyPart); 
        peakMessage.append(_id);
        peakMessage.append(isPeak);
//        peakMessage.append(note.pitch);
        msgs.push_back(peakMessage);
        return msgs;
    }
    
    
protected:
    //linear interpolation function
    double lerp(double a, double b, double f)
    {
        return a + f * (b - a);
    };
    
}; //end of  class

//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------

    //defining defaults here for now, including analysis window sizes, etc.
#define DEFAULT_FOOT_ONSET_STEP_COUNT_WINDOWSIZE 4 //in seconds
#define DEFAULT_FOOT_ONSET_STEP_COUNT_WINDOWSIZE_LONGER 45 //in seconds
    
    
    //TODO: add measure of how syncopated something is...
    class FootOnset : public SignalAnalysisEventOutput
    {
    protected:
        FindPeaks *leftFoot, *rightFoot;
        bool leftOnset, rightOnset;
        float stepsPerSampleSize;
        double curPeak, avgPeak;
        DataWindow stepPeaks;
        
        bool fakeStep;
        
        //    std::vector<float> stepTimes; // in seconds
        //    std::vector<float> stepPeaks; // in seconds
        
        //norm step count is 4 secs by default, longer window is steps, etc. over 45 secs by default.
        float window, windowLong;
        int stepCount, stepCountLong;
        int rightID, leftID;
        double timeSinceLastStep;
        
        //    void updateSteps(float curTime)
        //    {
        //        bool inWindow = false;
        //        while( stepTimes.size()>0 && !inWindow )
        //        {
        //            inWindow = stepTimes[0] >= ( curTime - window );
        //            if(!inWindow)
        //            {
        //                stepTimes.erase(stepTimes.begin());
        //                stepPeaks.erase(stepPeaks.begin());
        //            }
        //        }
        //    };
        
    public:
        enum MotionDataIndices { STEP_COUNT=0, AVG_PEAK=1, TIME_SINCE_LAST_STEP=2, STEP_COUNT_WINDOWSZ=3, STEP_COUNT_LONG=4, STEP_COUNT_WINDOWSIZE_LONG=5, IS_STEP=6, CUR_STEP_PEAK=7, STEP_LEFT=8, STEP_RIGHT=9,
            WHICH_AXIS_PEAK=10};
        
        FootOnset(FindPeaks *left, FindPeaks *right, int id1, int id2, float sampleSize = DEFAULT_FOOT_ONSET_STEP_COUNT_WINDOWSIZE, float sampleSizeLong = DEFAULT_FOOT_ONSET_STEP_COUNT_WINDOWSIZE_LONGER )
        :  SignalAnalysisEventOutput(NULL, 0, NULL)
        {
            leftFoot = left;
            rightFoot = right;
            leftOnset = false;
            rightOnset = false;
            window = sampleSize;
            stepsPerSampleSize = 0; //includes kicks... might be a problem with swinging legs
            stepCount = 0;
            avgPeak = 0;
            leftID = id1;
            rightID = id2;
            timeSinceLastStep = 0;
            windowLong = sampleSizeLong;
            
            stepPeaks.setWindowSize(sampleSizeLong);
            
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::STEP_COUNT));
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::AVG_PEAK));
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::TIME_SINCE_LAST_STEP));
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::STEP_COUNT_WINDOWSZ));
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::STEP_COUNT_LONG));
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::STEP_COUNT_WINDOWSIZE_LONG));
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::IS_STEP));
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::CUR_STEP_PEAK));
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::STEP_LEFT));
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::STEP_RIGHT));
            motionData.push_back(new MotionAnalysisEvent(MotionAnalysisDataType::DoubleEvent, MotionDataIndices::WHICH_AXIS_PEAK));
            
            
            
            motionData[MotionDataIndices::STEP_COUNT]->setName("Step Count");
            motionData[MotionDataIndices::AVG_PEAK]->setName("Avg. Peak");
            motionData[MotionDataIndices::STEP_COUNT_WINDOWSZ]->setName("Long window size");
            motionData[MotionDataIndices::STEP_COUNT_LONG]->setName("longer window step count");
            motionData[MotionDataIndices::STEP_COUNT_WINDOWSIZE_LONG]->setName("longer window");
            motionData[MotionDataIndices::IS_STEP]->setName("is step");
            motionData[MotionDataIndices::CUR_STEP_PEAK]->setName("current step accel. peak");
            motionData[MotionDataIndices::WHICH_AXIS_PEAK]->setName("which axis was the peak");
            
            
//            setMotionAnalysisMinMax(MotionDataIndices::STEP_COUNT, 0, 5 );
//            setMotionAnalysisMinMax(MotionDataIndices::AVG_PEAK, 0, 1);
//            setMotionAnalysisMinMax(MotionDataIndices::IS_STEP, 0, 1);
//            setMotionAnalysisMinMax(MotionDataIndices::CUR_STEP_PEAK, 0, 1);
//            setMotionAnalysisMinMax(MotionDataIndices::STEP_LEFT, 0, 1);
//            setMotionAnalysisMinMax(MotionDataIndices::STEP_RIGHT, 0, 1);
            
            
            //window size doesn't change so set it now
            ( (MotionAnalysisEvent *)motionData[MotionDataIndices::STEP_COUNT_WINDOWSZ])->setValue(sampleSize);
            ( (MotionAnalysisEvent *)motionData[MotionDataIndices::STEP_COUNT_WINDOWSIZE_LONG])->setValue(0);
            
        };
        
        virtual void updateMotionData()
        {
            motionData[MotionDataIndices::STEP_COUNT]->setValue(stepCount);
            motionData[MotionDataIndices::AVG_PEAK]->setValue(avgPeak);
            motionData[MotionDataIndices::TIME_SINCE_LAST_STEP]->setValue(timeSinceLastStep);
            motionData[MotionDataIndices::STEP_COUNT_LONG]->setValue(stepCountLong);
            motionData[MotionDataIndices::STEP_COUNT_WINDOWSIZE_LONG]->setValue(stepPeaks.curWindowSize()); //during first section, etc. will be not full-size
            motionData[MotionDataIndices::IS_STEP]->setValue(isStepping()); //during first section, etc. will be not full-size
            motionData[MotionDataIndices::CUR_STEP_PEAK]->setValue(curPeak); //accel peak of current or last recorded step
            motionData[MotionDataIndices::STEP_LEFT]->setValue(isLeftOnset()); //accel peak of current or last recorded step
            motionData[MotionDataIndices::STEP_RIGHT]->setValue(isRightOnset()); //accel peak of current or last recorded step
            
            int whichAxis;
            if( isRightOnset() && !isLeftOnset() )
                whichAxis = rightFoot->whichAxisWasPeak();
            else if( isLeftOnset() && !isRightOnset() )
                whichAxis = leftFoot->whichAxisWasPeak();
            else
            {
                whichAxis = std::max( leftFoot->whichAxisWasPeak(), rightFoot->whichAxisWasPeak() );
            }
            motionData[MotionDataIndices::WHICH_AXIS_PEAK]->setValue( whichAxis ); //accel peak of current or last recorded step
        };
        
        void update(float seconds)
        {
            stepPeaks.update(seconds);
            
            bool lastLeftOnset = leftOnset;
            
            leftOnset = leftFoot->getCombinedPeak();
            rightOnset = rightFoot->getCombinedPeak();
            
            if(fakeStep)
            {
                fakeStep = false;
                leftOnset = rightOnset = true;
                //peak will still be 0 or whatever
            }
            
            if( leftOnset && rightOnset )
            {
                if ( !lastLeftOnset )
                {
                    rightOnset = false;
                }
                else leftOnset = false;
            }
            
            if(leftOnset || rightOnset)
            {
                if( leftOnset )
                    curPeak = leftFoot->getCurrentPeak();
                else curPeak = rightFoot->getCurrentPeak();
                stepPeaks.push_back(curPeak, seconds);
            }
            if( stepPeaks.size() > 0 )
            {
                timeSinceLastStep = seconds - stepPeaks.lastTime() ;
            }
            
            stepCount = stepPeaks.size(seconds, window);
            stepCountLong = stepPeaks.size(seconds, windowLong);
            avgPeak = stepPeaks.getAverageOverWindow(window, seconds);
            if(isnan(avgPeak)) avgPeak = 0;
            //        std::cout << "avgPeak: " << avgPeak << std::endl;
            updateMotionData();
            //        std::cout << "after assign: " << motionData[1]->asDouble() << std::endl;
        };
        
        inline int getStepCount()
        {
            return stepCount;
        };
        
        inline bool isRightOnset()
        {
            return rightOnset;
        };
        
        inline bool isLeftOnset()
        {
            return leftOnset;
        };
        
        inline bool isStepping()
        {
            return ( rightOnset || leftOnset );
        };
        
        inline void createFakeStep()
        {
            fakeStep = true;
        }
        
        virtual std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            
            //don't send OSC... for now... maybe need to refactor this: CDB - 4/6/2015
            
            //        ci::osc::Message msg;
            //
            //        msg.setAddress(SHIMMER_STEP_DETECTION);
            //
            //        msg.addIntArg(leftID);
            //        msg.addIntArg(rightID);
            //
            //        msg.addIntArg( getStepCount() );
            //        msg.addIntArg( isLeftOnset() );
            //        msg.addIntArg( isRightOnset() );
            //        msg.addIntArg( isRightOnset() || isLeftOnset() );
            //        msg.addFloatArg(curPeak);
            //        msg.addFloatArg(timeSinceLastStep);
            //
            //
            //
            //        msgs.push_back(msg);
            
            return msgs;
        }
        
    };
    
    
    
}; //end of namespace

#endif /* PeakDetection_h */
