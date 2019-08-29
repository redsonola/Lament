//
//  BeatTiming.h
//  InteractiveTangoReadFromAndroid
//
//  Created by courtney on 11/5/14.
//
//

#ifndef InteractiveTangoReadFromAndroid_BeatTiming_h
#define InteractiveTangoReadFromAndroid_BeatTiming_h

#pragma once

#include "ReadCSV.h"

namespace CRCPMotionAnalysis  {
    
class BeatTimerSaveToFile
{
    
protected:
    std::string filename;
public:
        
BeatTimerSaveToFile(std::string f)
{
    std::stringstream ss;
    ss << f << "_beats.csv";
    filename = ss.str();
};
        
virtual void save(float seconds, int mNum, int beatNum, float s, float curb)
{
    std::ofstream myfile(filename, std::ios::app);
    myfile << seconds << "," << mNum << "," << beatNum << "," << s << "," << curb << std::endl;
    myfile.close();
};

};
    
class BeatTimerLoadFile
{
protected:
    std::vector<float> beatTimes;
    int m_index;
    float lastBeat;
    float m_modifiedTime;
public:
    BeatTimerLoadFile(std::string filename)
    {
        ReadCSV beatFile( filename );
        
        while ( !beatFile.eof() )
        {
            std::vector<std::string> tokens = beatFile.getTokensInLine();
            float beattime = std::atof(tokens[0].c_str());
            beatTimes.push_back(beattime);
        }
        
        beatFile.close();
        
        m_index = 0;
        lastBeat = 0;
        m_modifiedTime = 0;
    };
    
    void setModifiedTime(float mtime)
    {
        m_modifiedTime = mtime;
    };
    
    float getLastBeat(float seconds)
    {
        if (m_index < beatTimes.size())
        {
            while ( m_index < beatTimes.size() && beatTimes[m_index] < ( seconds + m_modifiedTime ) )
            {
                m_index++;
            }
        
            if( m_index < beatTimes.size() )
            {
                lastBeat = beatTimes[ m_index ];
            }
        }
        return lastBeat;
    };
};
    
class BeatTiming
{

public:
    enum class Beats {EIGHTH, SIXTEENTH, QUARTER};
    
    void setBPM(float tempo)
    {
        bpm = tempo;
        bps = tempo / 60.0;
        pulse8ps = bps / 8.0;
        pulse16ps = bps / 16.0;
        updateError();
        
    };
    
    void setWhichPulse(Beats pulse)
    {
        whichPulse = pulse;
        switch( whichPulse )
        {
            case Beats::EIGHTH:
                curPulse = pulse8ps;
                break;
            case Beats::SIXTEENTH:
                curPulse = pulse16ps;
                break;
            case Beats::QUARTER:
                curPulse = 0;
                break;
            default:
                break;
        }
        
    };
    
    float getPulse16()
    {
        return pulse16ps;
    };
    
    float getPulse8()
    {
        return pulse8ps;
    };
    
    void setError(float err) // in percentage of a 16 note (for now)
    {
        percentError = err;
        updateError();
    };
    
    BeatTiming(float tempo = 100.0, float err = 0.25)
    {
        setBPM(tempo);
        setError(err);
        lastBeat = 0;
        setWhichPulse( Beats::EIGHTH );
        saver = NULL;
        loader = NULL;
        curTimeInSec = 0;
    };
    
    float getBPM()
    {
        return bpm; 
    };
    
    void setLastBeat(float secs, int mNum, int beatNum, float s, float curb)
    {
        lastBeat = secs;
         whichBar = mNum;

        
        //save a beat
        if(saver!=NULL && onBeat)
        {
            saver->save(secs, mNum, beatNum, s, curb);
        }
    }
    
    long currentBar()
    {
        return whichBar;
    };
    
    void save(std::string filename)
    {
        saver = new BeatTimerSaveToFile(filename);
    };
    
    void load(std::string filename)
    {
        loader = new BeatTimerLoadFile(filename);
    };
    
    void update( float curTime )
    {
//        updateLastBeat(curTime);
        
        if( loader != NULL)
        {
            lastBeat = loader->getLastBeat( curTime );
        }
        curTimeInSec = curTime;
        float minTime = lastBeat - marginOfError;
        float maxTime = lastBeat + marginOfError;
        onBeat =  ( ( curTime + modifiedTime ) >= minTime && ( curTime + modifiedTime ) <= maxTime);
//        if( onBeat ) std::cout << "BEAT\n";
        
    };
    
    
    
    bool isOnBeat()
    {
//        if( onBeat ) std::cout << "BEAT\n";
        return onBeat;
    };
    
    bool isOnBeat(float err, float curTime) //for custom error percentages
    {
        float minTime = lastBeat - err + modifiedTime;
        float maxTime = lastBeat + err + modifiedTime;
        
//        if( onBeat ) std::cout << "BEAT: " << "lastBeat: " << lastBeat << modifiedTime <<
//            " curTime: " << curTime << std::endl ;

        
        return ((curTime+modifiedTime) >= minTime && (curTime+modifiedTime) <= maxTime);
    };

    float getMarginOfError()
    {
        return marginOfError;
    };
    double getTimeInSeconds()
    {
        return curTimeInSec+modifiedTime;
    };

    void setModifiedTime(float mtime)
    {
        modifiedTime = mtime;
        if( loader != NULL )
        {
            loader->setModifiedTime(mtime);
        }
    };
    
protected:
    float bps ;
    float bpm;
    float pulse8ps; //in seconds
    float pulse16ps; //in seconds
    float curPulse;
    float marginOfError;
    float percentError;
    float lastBeat; //(in seconds)
    float onBeat;
    double curTimeInSec;
    float modifiedTime;
    long  whichBar;
    
    Beats whichPulse;
    BeatTimerSaveToFile *saver;
    BeatTimerLoadFile *loader;
    
    void updateError()
    {
        marginOfError = pulse16ps * percentError;
    };
    
    void updateLastBeat(float curTime)
    {
        curTimeInSec = curTime;
        if( lastBeat + curPulse >= curTime  )
        {
            {
                lastBeat += pulse8ps;
            }
        }
    }

    
};


};

#endif
