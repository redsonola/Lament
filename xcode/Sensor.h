//
//  Sensor.h
//  VideoAndOSCLab
//
//  Created by courtney on 10/22/18.
//  Represents a sensor. Allows an id from a sensor in order to keep track of it.
//

#ifndef Sensor_h
#define Sensor_h

namespace CRCPMotionAnalysis {
    
    
#define SENSORDATA_BUFFER_SIZE 1024


class SensorData
{
public:
    SensorData(std::string deviceID, int which)
    {
        setDeviceID(deviceID);
        setWhichSensor(which);
//        setDancerLimb(0, 0);
//        setPareja(0);
        curNumAdded = 0;
    };
    
    inline int getWhichSensor()
    {
        //this is returning which sensor it is according to android/shimmer setup
        return whichSensor;
    };
    
    inline std::string getDeviceID()
    {
        return mDeviceID;
    };
    
    inline void setDeviceID(std::string idz)
    {
        mDeviceID = idz;
    };
    
    bool same( std::string deviceID, int sensor )
    {
        return ( !deviceID.compare( mDeviceID ) && sensor == whichSensor );
    };
    
    inline void setWhichSensor(int sensor)
    {
        whichSensor = sensor;
    };
    
    void addSensorData( MocapDeviceData *data )
    {
        mSensorData.push_back(data);
    };
    
    void eraseData()
    {
        mSensorData.clear();
    };
    
    virtual void update(float seconds)
    {
        addToBuffer(mSensorData);
        curNumAdded = mSensorData.size();

    };

    virtual std::vector<MocapDeviceData *> getBuffer( int bufferSize = 25 )
    {
        if( bufferSize > mBuffer.size() )
        {
            return mBuffer;
        }
        std::vector<MocapDeviceData *>::iterator startIter = mBuffer.end() - bufferSize;
        std::vector<MocapDeviceData *>::iterator endIter = mBuffer.end();
        
        return std::vector<MocapDeviceData *>( startIter, endIter );
    };
    
    //this gets all the new samples from the buffer
    //then it puts all the samples that have the same time-stamp into one Shimmer data value
    //    virtual std::vector<ShimmerData *> getNewIntegratedSamples()
    //    {
    //
    //
    //    };
    
    virtual inline int getNewSampleCount()
    {
        return curNumAdded;
    };
    
    virtual void resetPlaybackTimer() //TODO
    {
        
    };
    
    virtual void cleanupBuffer()
    {
        
        if( mBuffer.size() > SENSORDATA_BUFFER_SIZE )
        {
            int numErase = mBuffer.size() - SENSORDATA_BUFFER_SIZE;
            for(int i=0; i<numErase; i++ )
            {
                if( mBuffer[i] != NULL)
                    delete mBuffer[i];
                mBuffer[i] = NULL;
            }
            mBuffer.erase( mBuffer.begin(), mBuffer.begin()+numErase );
        }
    };
    
//    void setDancerLimb(int dancer, int limb)
//    {
//        whichDancer = dancer;
//        whichLimb = limb;
//    };
//
//    void setPareja(int pareja)
//    {
//        whichPareja = pareja;
//    };
    
protected:
    std::vector<MocapDeviceData *> mSensorData;
    std::vector<MocapDeviceData *> mBuffer; //keep a buffer data
    std::string mDeviceID;
    int whichSensor;
    int curNumAdded;
    
    int whichDancer;
    int whichLimb;
    
    // BUFFER_SIZE
    virtual void addToBuffer( std::vector<MocapDeviceData *> data )
    {
        
        mBuffer.reserve( data.size() + mBuffer.size() ); // preallocate memory
        mBuffer.insert( mBuffer.end(), data.begin(), data.end() );
        
    };
    
};
    
}

#endif /* Sensor_h */
