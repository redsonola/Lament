//
//  MotionAnalysisOuput.h
//  InteractiveTangoReadFromAndroid
//
//  Created by courtney on 12/2/14.
//
//

#ifndef InteractiveTangoReadFromAndroid_MotionAnalysisOuput_h
#define InteractiveTangoReadFromAndroid_MotionAnalysisOuput_h

namespace CRCPMotionAnalysis
{

    enum MotionAnalysisDataType { Generic=0, IntEvent=1, FloatEvent=2, DoubleEvent=3 };
    
class MotionAnalysisData
{
protected:
    int dataID;
    std::string dataName;
    
    double val;
    double minVal, maxVal;

public:
    MotionAnalysisData(int idz)
    {
        setID(idz);
        
        //default
        maxVal = 1;
        minVal = 0;
    };
    
    virtual MotionAnalysisDataType getType()
    {
        return Generic ;
    };
    
    int getID()
    {
        return dataID;
    };

    std::string getName()
    {
        return dataName;
    };
    
    void setID(int idz)
    {
        dataID = idz;
    };
    
    void setName(std::string name)
    
    {
        dataName = name;
    };
    
    int asInt() { return ((int)val); };
    float asFloat() { return ((float)val); };
    double asDouble() { return val; };
    void setValue(int v){ val = v; };
    void setValue(float f){ val = f;};
    void setValue(double d){ val = d;};
    virtual void setMinMax(double minv, double maxv)
    {
        minVal = minv;
        maxVal = maxv;
    };
    
    double scaledValue() //returns in range of 0..1
    {
        double newVal =   ( val - minVal) / ( maxVal - minVal );
        //        newVal = std::min(newVal, maxVal);
        //        newVal = std::max(newVal, minVal);
        return newVal;
    };
};

//stores everything as a double but allows other forms of precision
class MotionAnalysisEvent : public MotionAnalysisData
{
protected:
    MotionAnalysisDataType whichType;
    std::vector<double> coefficients;  //the co-efficients for the polynomial fit curve for the scaling
    
public:
    MotionAnalysisEvent(MotionAnalysisDataType ma, int idz) : MotionAnalysisData(idz)
    {
        whichType = ma;
        val = NO_DATA;
    };
    
    MotionAnalysisEvent(float f, int idz) : MotionAnalysisData(idz)
    {
        val = f;
        whichType = FloatEvent;
    };
    
    MotionAnalysisEvent(double d, int idz) : MotionAnalysisData(idz)
    {
        val = d;
        whichType = DoubleEvent;
    };
    
    MotionAnalysisEvent(int i, int idz) : MotionAnalysisData(idz)
    {
        val = i;
        whichType = IntEvent;
    };
    
    virtual MotionAnalysisDataType getType()
    {
        return whichType ;
    };
    
    void setCoEfficients(std::vector<double> coeffs)
    {
        coefficients = coeffs;
    };
    
    virtual void setMinMax(double minv, double maxv)
    {
        coefficients.clear();
        minVal = minv;
        maxVal = maxv;
        coefficients.push_back(minVal);
        coefficients.push_back(maxVal);
    };


    void setMin(double m)
    {
        minVal = m;
    };
    void setMax(double m)
    {
        maxVal = m;
    };
    double getMin()
    {
        return minVal;
    };
    double getMax()
    {
        return maxVal; 
    };
    

    
    //this is a hack... should switch if moving back to scaled values
    //TODO: FIX
    double scaledValuePolyFit()
    {
        double sum = 0;

        if( coefficients.size() > 2  )
        {
            int degree = coefficients.size() - 1;
            for(int i=0; i<coefficients.size(); i++)
            {
                sum += coefficients[i]*std::pow(val, degree);
                degree--;
            }
        }
        else
        {
            minVal = coefficients[0];
            maxVal = coefficients[1];
            sum = scaledValue();
        }
        
        //cap at 0-1
        if(sum > 1) sum = 1;
        if(sum < 0) sum = 0;
        
//        if(!getName().compare("FOLLOWER_STEPNUM"))
//            std::cout << getName() << " --> sum: " << sum << "co-efficient size: " <<  coefficients.size() <<  " minval: " << minVal << " maxVal " << maxVal << std::endl;

            
//            std::cout << getName() << " --> sum: " << sum << std::endl;

        return sum;
    };


    
};


    
};

#endif
