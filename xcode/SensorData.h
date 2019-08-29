//
//  SensorData.h
//  InteractiveTangoReadFromAndroid
//
//  Created by courtney on 9/11/14.
//
//

#ifndef InteractiveTangoReadFromAndroid_SensorData_h
#define InteractiveTangoReadFromAndroid_SensorData_h

#include "ReadCSV.h"

#define SENSORDATA_BUFFER_SIZE 3100 //uhhh yal
#define REAL_TIME_SENSOR 0
#define LOADER_SENSOR 1

class OSCSaveToFile : public ShimmerSaveToFile
{
public:
    
    OSCSaveToFile(std::string f, std::string s ) : ShimmerSaveToFile(f, s)
    {
    };
    
	virtual void save(std::vector<ShimmerData *> data, float seconds)
	{
		std::ofstream myfile(filename, std::ios::app);
		for (int i = 0; i < data.size(); i++)
		{
            data[i]->setData(19, seconds);
            
			myfile << dataIndex << ",";
			for (int j = 0; j < SHIMMER_NUM_ARGS+1; j++)
			{
				myfile << data[i]->getData(j);
				myfile << ",";
			}
            for(int j=0; j<4; j++)
            {
                myfile << data[i]->getQuarternion(j) << ",";
            }
            for(int j=0; j<3; j++)
            {
                myfile << data[i]->getOrientationMatrix(j);
                if(j<2) myfile << ",";
            }
			myfile << std::endl;
			dataIndex++;
		}
        
		myfile.close();
	};
};

class OSCLoadFromFile : public ShimmerLoadFromFile
{
protected:
    float speed;
public:
    OSCLoadFromFile( std::string filename ) : ShimmerLoadFromFile()
    {
        speed = 1;
		index = -1;
        modifier_num = 0;
        looping = -1;
        
		//get the port number
		int findex, gindex = 0;
		findex = filename.find_first_of(".");
		gindex = filename.find(".", findex + 1);
		port = filename.substr( findex + 1, gindex - findex - 1 );
        
		//load from file now
//        ShimmerLoadFromFile::loadFromFile(filename);
//        for( int i=0; i<sData.size(); i++ )
//            sData[i]->setData(0, i);
        
        loadFromFile(filename);
        index = 0;
    };
    
    inline void setSpeed(float s)
    {
        speed = s;
    }
    
    virtual void loadFromFile(std::string filename)
	{
        
        ReadCSV csvReader(filename);
        int csvIndex = 0;
        
        while( !csvReader.eof() )
        {
            std::vector<std::string> tokens = csvReader.getTokensInLine();
            ShimmerData *shimmer = new ShimmerData;
            
            shimmer->setData(0, csvIndex);

            int i;
            for (i = 1; i < SHIMMER_NUM_ARGS+1; i++) //plus one for cam.
            {
                shimmer->setData(i, std::atof(tokens[i].c_str()));
            }
            if( tokens.size() > 20 )
            {
                float quat[4], oVals[3];
                for(int j=0; j<4; j++)
                {
                    quat[j] = std::atof(tokens[i].c_str());
                    i++;
                }
                for(int j=0; j<3; j++)
                {
                    oVals[j] = std::atof(tokens[i].c_str());
                    i++;
                }
                shimmer->setOrientationMatrix(oVals[0], oVals[1], oVals[2]);
                shimmer->setQuarternion(quat[0], quat[1], quat[2], quat[3]);
            }
            sData.push_back(shimmer);
            csvIndex++;
        }
        csvReader.close();
	};
    
    std::vector<ShimmerData *> getData(float seconds)
	{
		assert(index > -1 && index < sData.size());
        
		std::vector<ShimmerData *> shimmers;
        float stamp = sData[index]->getCameraTimeStamp();
        
        if( ((seconds*speed)+modifier_num) > looping && looping > -1)
        {
            reset();
        }
        
        while (index < sData.size() && stamp <= ((seconds+modifier_num) ) )
        {
            stamp = sData[index]->getCameraTimeStamp();
            shimmers.push_back(sData[index]);
            index++;
        }
		return shimmers;
	};
    
    int getCurSample()
	{
		return index;
	};

};

class SensorData
{
public:
    SensorData(std::string deviceID, int shimmer, VisualizeShimmer *v = NULL)
    {
        setDeviceID(deviceID);
        setWhichShimmer(shimmer);
        saveFile = NULL;
        setVisualizer(v);
        setDancerLimb(0, 0);
        setPareja(0);
    };
    
    inline void setPort(std::string p)
    {
        port = p;
    };
    
    inline std::string getPort()
    {
        return port; 
    };
    
    inline int getWhichSensor()
    {
        //this is returning which sensor it is according to android/shimmer setup
        return whichShimmer; 
    };
    
    inline std::string getDeviceID()
    {
        return mDeviceID;
    };
    
    inline void setDeviceID(std::string idz)
    {
        mDeviceID = idz;
    };
    
    bool same( std::string deviceID, int shimmer )
    {
        return ( !deviceID.compare( mDeviceID ) && shimmer == whichShimmer );
    };
    
    inline void setWhichShimmer(int shimmer)
    {
        whichShimmer = shimmer;
    };
    
    void addShimmer( ShimmerData *data )
    {
        mShimmerData.push_back(data);
    };
    
    void saveToFile( std::string f )
    {
        std::stringstream ss;
        //filename thus has: deviceID, shimmerID, pareja, dancer, limb information
        ss << mDeviceID << "__" << whichShimmer << "__" << whichPareja << "__" << whichDancer << "__" <<whichLimb;
        
        if(saveFile==NULL)
            delete saveFile; 
            
        saveFile = new OSCSaveToFile( f,  ss.str());
    };
    
    void updateVisualizer()
    {
        if (visualizer != NULL)
            visualizer->update(mShimmerData);
    }
    
    void save(float seconds)
    {
        if(saveFile != NULL)
        {
            saveFile->save( mShimmerData, seconds );
        }
        mShimmerData.clear();
    };
    
    void eraseData()
    {
        mShimmerData.clear();
    };
    
    inline void setVisualizer(VisualizeShimmer *v){ visualizer = v; };
    
    virtual void update(float seconds)
    {
        updateVisualizer();
        addToBuffer(mShimmerData);
        curNumAdded = mShimmerData.size();

        //TODO: FIX -- this change is probably why it crashed on the save -- NOTE CDB -- 11/6/2017
//        saveWorkerThread = boost::thread(&SensorData::save, this, seconds);
//        save(seconds);
    }; //maybe will use later
    
    virtual inline void start()
    { //dummy
    };
    
    virtual std::vector<ShimmerData *> getBuffer( int bufferSize = 25 ) // check this in git diff -- 11/6/2017
    {
        if( bufferSize > mBuffer.size() )
        {
            return mBuffer;
        }
        std::vector<ShimmerData *>::iterator startIter = mBuffer.end() - bufferSize;
        std::vector<ShimmerData *>::iterator endIter = mBuffer.end();

        return std::vector<ShimmerData *>( startIter, endIter );
    };
    
    virtual inline int getNewSampleCount()
    {
        return curNumAdded;
    };
    
    virtual void resetPlaybackTimer()
    {
        
    };

    virtual int getType()
    {
        return REAL_TIME_SENSOR;
    };
    
    
    virtual void cleanupBuffer()
    {
        
        if( mBuffer.size() > SENSORDATA_BUFFER_SIZE )
        {
            int numErase = mBuffer.size() - SENSORDATA_BUFFER_SIZE;
            for(int i=0; i<numErase; i++ )
            {
                delete mBuffer[i];
            }
            mBuffer.erase( mBuffer.begin(), mBuffer.begin()+numErase );
        }
    };
    
    void setDancerLimb(int dancer, int limb)
    {
        whichDancer = dancer;
        whichLimb = limb;
    };
    
    void setPareja(int pareja)
    {
        whichPareja = pareja;
    };
    
protected:
    std::vector<ShimmerData *> mShimmerData;
    std::vector<ShimmerData *> mBuffer; //keep a buffer data
    std::string mDeviceID;
    std::string port;
    int whichShimmer;
    OSCSaveToFile *saveFile;
    VisualizeShimmer *visualizer;
    int curNumAdded;
    
//    boost::thread saveWorkerThread;

    
    int whichDancer;
    int whichPareja;
    int whichLimb;
    
    // BUFFER_SIZE
    virtual void addToBuffer( std::vector<ShimmerData *> data )
    {
        
        mBuffer.reserve( data.size() + mBuffer.size() ); // preallocate memory
        mBuffer.insert( mBuffer.end(), data.begin(), data.end() );

    };

};

class LoadedSensor : public SensorData
{
public:
    
    LoadedSensor(std::string filename, std::string deviceID, int which,
                 VisualizeShimmer *v = NULL) : SensorData(deviceID, which, v)
    {
        oscLoader = new OSCLoadFromFile(filename);
    };
    
    virtual void update(float seconds)
    {
        assert(oscLoader != NULL);
        
		if (oscLoader->started() && !oscLoader->done())
		{
 			std::vector<ShimmerData *> shimmers = oscLoader->getData(seconds);
			for (int i = 0; i < shimmers.size(); i++)
			{
				mShimmerData.push_back(shimmers[i]);
			}
		}
	
        if (visualizer != NULL)
            visualizer->update(mShimmerData);
        
        updateVisualizer();
        addToBuffer(mShimmerData);
        curNumAdded = mShimmerData.size();

    }
    
    virtual void resetPlaybackTimer()
    {
        oscLoader->reset();
    };
    
    virtual void start()
    {
        oscLoader->start();
    }
    
    virtual void stop()
    {
        oscLoader->stop();
    }
    
    virtual void setSpeed(float s)
    {
        oscLoader->setSpeed(s);
    }
    
    virtual int getType()
    {
        return LOADER_SENSOR;
    };

    void setTimeModifierNum(float modTime)
    {
        assert(oscLoader != NULL);
        oscLoader->setTimeModifierNum(modTime);
    };
    
    int getCurSample()
    {
       return oscLoader->getCurSample();
    }
    
protected:
    std::string filename;
    OSCLoadFromFile *oscLoader;
    
};

#endif
