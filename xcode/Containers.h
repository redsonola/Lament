//
//  Containers.h
//  InteractiveTangoReadFromAndroid
//
//  Created by courtney on 5/12/15.
//
//

#ifndef InteractiveTangoReadFromAndroid_Containers_h
#define InteractiveTangoReadFromAndroid_Containers_h


namespace CRCPMotionAnalysis  {

    
//utility class -- commented out bc not used
//class CircularIndex
//{
//private:
//    int val;
//public:
//    int max;
//    int min;
//    int value(){ return val; };
//    CircularIndex(int mi=0, int ma=0 )
//    {
//        setMinMax(mi, ma);
//    };
//    void setMinMax(int mi, int ma )
//    {
//        val = mi;
//        min = mi;
//        max = ma;
//    };
//    void operator++()
//    {
//        val++;
//        if( val > max )
//        {
//            val = min;
//        }
//    };
//    bool atMax()
//    {
//        return ( val == max );
//    };

//};
    

//maybe put this in a different header file at a later point!!!!!!
//TODO: refactor... should be in foot onset class, etc.
class DataWindow
{
    //a buffer with a max length in seconds, not # of data... discards data outside the time window
protected:
    std::vector<double> _data;
    std::vector<double> times; //time stamp of ea data
    double windowSize;
//    float cTime; //current time
    
public:
    DataWindow(double window =2)
    {
        windowSize = window;
    };
    
    void setWindowSize(double sz)
    {
        windowSize = sz;
    };
    
    void update(float curTime)
    {
        bool inWindow = false;
        while( times.size()>0 && !inWindow )
        {
            inWindow = times[0] >= ( curTime - windowSize );
            if(!inWindow)
            {
                times.erase(times.begin());
                _data.erase(_data.begin());
            }
        }
    };
    
    void push_back(double d, double secs)
    {
        _data.push_back(d);
        times.push_back(secs);
    };
    
    //sum over current data values
    double getSum()
    {
        double sum = 0;
        for(int i=0; i<_data.size(); i++)
        {
            sum += _data[i];
        }
        return sum;
    };
    
    //average over current data values
    double getAvg()
    {
        return ( ( getSum() ) / ( double(_data.size() ) ) );
    };
    
    //quick fix -- so fix
    double getAvgOverSample(int sampNum)
    {
        double sum = 0;
        for(int i=0; i< sampNum; i++)
            sum += _data[_data.size() -1 -i];
        
//        std::cout << "mood is: " << sum/(double)sampNum << std::endl;
        return sum/(double)sampNum;
    }
    
    
    //TODO: get a subsection of sections from a buffer... via seconds
    //TODO: CHECK THIS SHIT.
    std::pair<double, int> getSumOverWindow(double secondWindow, double curTime)
    {
//        assert( secondWindow <= windowSize );
        if( secondWindow == windowSize )
        {
            
           return std::pair<double, int>(getSum(), _data.size() );
        }
        else
        {
            if( secondWindow > windowSize )
                std::cout << "Warning! Asked for a larger window size than exists in this data window!\n";
            
            bool inWindow = true;
            double sum = 0;
            int i = times.size();
            while( i > 0  && inWindow )
            {
                i--;
                inWindow = times[i] >= ( curTime - secondWindow );
                if (inWindow) sum += _data[i];
            }
            int count = _data.size() - (i+1);
            std::cout << "in proc -- sum: " <<sum << " count: " << count << std::endl;

            return std::pair<double, int>(sum, count);
          //if empty, still returns 1, but this should be fine for all applications, esp. since there will be a /
        }
    };
    
    double getAvgOverWindow(double secondWindow, double curTime)
    {
        //        assert( secondWindow <= windowSize );
        if( secondWindow == windowSize )
        {
            
            return getSum()/( (double) _data.size() );
        }
        else
        {
            if( secondWindow > windowSize )
                std::cout << "Warning! Asked for a larger window size than exists in this data window!\n";
            
            bool inWindow = true;
            double sum = 0;
            int i = times.size();
            while( i > 0  && inWindow )
            {
                i--;
                inWindow = times[i] >= ( curTime - secondWindow );
                if (inWindow) sum += _data[i];
            }
            int count = _data.size() - (i+1);
//            std::cout << "in proc -- sum: " <<sum << " count: " << count << std::endl;
            
            if( count == 0 ) return 0;
            else
                return sum / ( (double) count);
            //if empty, still returns 1, but this should be fine for all applications, esp. since there will be a /
        }
    };
    
    
    double getAverageOverWindow(double secondWindow, double curTime)
    {
        if( secondWindow == windowSize ) return getAvg();
        
        return getAvgOverWindow(secondWindow, curTime);

    };
    
    //get last value added
    double top()
    {
        //for now, an assert... could also do a no_data constant but not using this yet, so...
        assert( _data.size() > 0 );
        return _data[ _data.size()-1 ];
    };
    
    double lastTime()
    {
        //for now, an assert... could also do a no_data constant but not using this yet, so...
        if( times.size() > 0 )
            return times[ times.size()-1 ];
        else return 0;
    };
    
    double at(int i, float seconds=0, float secondWindow=0)
    {
        assert( i > 0 && i < size());
        
        if( secondWindow == 0 )
            return _data[i];
        else
        {
            int count = size(seconds, secondWindow);
            int zeroForWindow = times.size() - count;
            int index = zeroForWindow + i;
            return _data[index];
        }
    };
    
    virtual size_t size(float curTime=0, float secondWindow=0)
    {
        if( secondWindow == 0 || windowSize == secondWindow )
            return times.size();
        if( secondWindow > windowSize )
        {
            std::cout << "Warning! Asked for a larger buffer than is available!! Returning max size buffer! \n";
            return size();
        }
        else
        {
            bool inWindow = true;
            size_t count = 0;
            int i = times.size();
            while ( i>0 && inWindow)
            {
                i--;
                inWindow = times[i] >= ( curTime - secondWindow );
                if( inWindow ) count++;
            }
            return count;
        }
        
    };
    
    float curWindowSize() //returns actual seconds of window... music may not have been going long enough to fill the window
    {
        if( times.size() < 2 ) return 0;
        else
            return lastTime() - times[0];
    };

};
    
    //TODO: put this somewhere else .. this just a useful method... where to put??-- also... a bit hack.
    //input: in_val -- value to be scales
    // (x1,y1), (x2,y2) -- points that the lines pass through
    
    double exp_scale(double in_val, double x1=0.1, double y1=0.1, double x2=10, double y2=10 )
    {
        in_val = in_val + 0.0000000001; //no zeros -- HACK.
        double b = log10(y1/y2) / (x1-x2);
        double a = y1 / pow(10.0,( b * x1 ));
        double y = a * pow(10.0,( b * in_val ));
        return y;
    };
    
};



#endif
