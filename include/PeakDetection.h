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


#define MIN_PEAK_WINDOW 36 //should define in relation to the sample rate
    
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
    float _id;
    
    Derivative derivative; //we need this to find the peaks according to matlab func. using
    
public:
    FindPeaks( float waitBetween, SignalAnalysis *s, int id_, int bufferSize=48) : SignalAnalysis( s, bufferSize, NULL ), derivative(s, buffersize)
    {
        combinedPeak = false;
        _id = id_;
        
        //TODO: & note--> this can be changed with a bpm, if that changes -- I have code for this, if relevant
        waitBetweenPeaks = waitBetween;
        peakTimer.setWaitTime( waitBetweenPeaks ); //eg, for the space of an 8th or 16th note... depends if you are using this for step detection
        
        //these values don't mean anything unless comnbinedPeak is true
        curPeakHeight = 0; //current peak, of all axes
        whichAxisPeak = MocapDeviceData::DataIndices::ACCELX; //just a default
        
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

        //a hack to compensate for different sample rate
        int windowSize;
//        if(data1[0]->getDeviceType()==MocapDeviceData::MocapDevice::IPHONE) //assume all data comes from the same device
            windowSize = data1.size() - std::min(getNewSampleCount(), int(SR/4)) - 30; //really where we start in the data buffer
//        else
//        {
//            windowSize = data1.size() - 210; //really where we start in the data buffer
//        }

        
        bool FoundPeakX = false; //have not found the peak yet
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
        SignalAnalysis::update(seconds);
        derivative.update();
        data1 = derivative.getBuffer();
        
        if( data1.size() < MIN_PEAK_WINDOW ) return ; //this is only useful if there is enough of a buffer

        if( getNewSampleCount() <=0 ) return ; //don't bother if no new samples
        combinedPeak = false;
        peakTimer.update(seconds);
        curPeakHeight = NO_DATA;
        
        double peakx=0, peaky=0, peakz=0;
        
        combinedPeak = ( findIC(MocapDeviceData::DataIndices::ACCELX, peakx, xThresh) || findIC(MocapDeviceData::DataIndices::ACCELY, peaky, yThresh) || findIC(MocapDeviceData::DataIndices::ACCELZ, peakz, zThresh) );
        
        //we only count peaks that occur far enough apart to be considered a separate event
        combinedPeak = ( combinedPeak ) && ( peakTimer.readyForNextPeak() );

        if(combinedPeak) //reset peak timer
            peakTimer.peakHappened();
        
        curPeakHeight = findNormalizedPeakHeight(peakx, peaky, peakz); //find how big the peak was.
    };
    
    virtual std::vector<ci::osc::Message> getOSC()
    {
        
        std::vector<ci::osc::Message> msgs;
        
        //TODO: add OSC messages here -- eg. if peak is true create and send a message
        
        //this tests the peak -- the count is to differentiate it from the last peak
        int isPeak = getCombinedPeak();
        ci::osc::Message peakMessage;
        peakMessage.setAddress(NOTCH_PEAK);
        peakMessage.append(_id);
        peakMessage.append(isPeak);
        
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

}; //end of namespace

#endif /* PeakDetection_h */
