//
//  Instruments.h
//  InteractiveTangoReadFromAndroid
//
//  Created by courtney on 7/28/16.
//
//

#ifndef InteractiveTangoReadFromAndroid_Instruments_h
#define InteractiveTangoReadFromAndroid_Instruments_h

#endif

namespace CRCPMotionAnalysis
{
    
    //files for instruments
#define INSTRUMENTS_FILE "/Users/courtney/Documents/Dissertation Work - Interactive Tango/song_data/Instruments.csv"


class Instrument
{
protected:
    int instrumentID;
    std::string nameStr;
    std::vector<int> profile;
    int melTrackID; //which track in ableton is this instrument rounted through?
public:
    
    Instrument(int instrID, std::string _name)
    {
        instrumentID = instrID;
    };
    
    void addProfile(int profileID)
    {
        profile.push_back( profileID );
    };
    
    void setMelTrackID(int id_)
    {
        melTrackID = id_;
    };
    
    int getTrackID()
    {
        return melTrackID;
    };
    
    int getInstrumentID()
    {
        return instrumentID;
    };
    
    bool fitsProfile( std::vector<int> _profile )
    {
        assert( _profile.size() == profile.size());
        bool fits = true;
        int index = 0;
        while ( fits && index < profile.size() )
        {
            fits = profile[index] == _profile[index];
            index++;
        }
        
        return fits;
    };
    
    int getProfile(int i)
    {
        assert( i < profile.size() && i >= 0 );
        return profile[i];
    };
    
    int getProfileSize(){ return profile.size(); };
};
    
class Orchestra
{
protected:
    std::vector<int> profile;
    std::vector<Instrument *> instruments;
public:
    Orchestra(){};
    void addInstrument( Instrument *instr  )
    {
        assert(profile.size() == instr->getProfileSize() );
        instruments.push_back(instr);
        updateProfile();
    };
        
    void updateProfile()
    {
        int profileSize = profile.size();
        if( profileSize == 0 )
        {
            for( int i=0; i<instruments[0]->getProfileSize(); i++ )
            {
                profile.push_back( instruments[0]->getProfile(i) );
            }
        }
        else
        {
            
            profile.clear();
            for( int i=0; i<profileSize; i++ )
            {
                profile.push_back(0);
                for( int j=0; j<instruments.size(); j++ )
                {
                    profile[i] +=  instruments[j]->getProfile(i);
                }
                profile[i] = std::round(double(profile[i] / double(instruments.size())));
            }
        }
    }
    
    //this fudges for a middle value bc instruments are either min or max, so one instrument responding to a middle value can choose any one solo instrument
    //FOR NOW!
    bool fitsProfile( boost::shared_ptr<std::vector<int>> profile_, bool fudgeMiddleIfOnlyOneInstrument = true, int minP = 1, int maxP = 3 )
    {
        assert( profile_->size() == profile.size() );
        int index = 0;
        bool fits = true;
        if( instruments.size() > 1 || (!fudgeMiddleIfOnlyOneInstrument)  )
        {
            while ( fits && index < profile.size() )
            {
                fits = profile[index] == profile_->at(index);
                index++;
            }
        }
        else
        {
            int middleVal = std::round(double(( minP + maxP ))/ 2.0);
            while ( fits && index < profile.size() )
            {
                fits = ( profile[index] == profile_->at(index) ) || ( profile_->at(index) == middleVal );
                index++;
            }
        }
        
        return fits;
    };
    
    bool fitsProfileExcludingTHESEInstruments(Orchestra *orch, boost::shared_ptr<std::vector<int>> profile_, bool fudgeMiddleIfOnlyOneInstrument = true, int minP = 1, int maxP = 3  )
    {
        assert( profile_->size() == profile.size() );
        bool fits = true;
        
        //if the only instruments are the ones excluded, say that it fits, defacto. Of course, this shouldn't happen at this point
        if( size() == orch->size() )
        {
            std::cout << "Warning! Tried to for profile fit in orchestration but only have excluded instruments!\n";
            return fits;
        }
        
        //find the profile of instruments w/o the ones excluded
        std::vector<int> profileForCheck;
        double instrSize=0;
        for(int j=0; j<profile_->size(); j++)
        {
            profileForCheck.push_back(0);
            for(int i=0; i< size(); i++)
            {
                if( !orch->includesInstrument(instruments[i]) )
                {
                    profileForCheck[j] += instruments[i]->getProfile(j);
                    instrSize++;
                }
            }
            profileForCheck[j] = std::round(double(profileForCheck[j]) / instrSize);
        }
        
        //compare that profile with profile given
        int index = 0;
        if( instrSize > 1 || !fudgeMiddleIfOnlyOneInstrument  )
        {
            while ( fits && index < profileForCheck.size() )
            {
                fits = profileForCheck[index] == profile_->at(index);
                index++;
            }
        }
        else
        {
            int middleVal = std::round(double(( minP + maxP ))/ 2.0);
            while ( fits && index < profileForCheck.size() )
            {
                fits = ( profileForCheck[index] == profile_->at(index) ) || ( profile_->at(index) == middleVal );
                index++;
            }
        }
        
        return fits;
    };
    
        
    bool includesInstrument( int instrumentID )
    {
        return getInstrument(instrumentID) != NULL;
    };
    
    bool includesInstrument(Instrument *ins)
    {
        return includesInstrument(ins->getInstrumentID());
    };
        
    virtual Instrument *getInstrument(int id_)
    {
        Instrument *instr =  NULL;
        int index = 0;
        
        while( index < instruments.size() && instr == NULL )
        {
            if( instruments[index]->getInstrumentID() == id_ ) instr = instruments[index];
            index++;
        }
        
        return instr;
    }
        
    size_t size()
    {
        return instruments.size();
    };
    
    Instrument *getInstrViaIndex(int index)
    {
        assert( index < size() && index >= 0 );
        return instruments[index];
    };
    
    void clear()
    {
        instruments.clear();
        profile.clear();
    };
    
    void addfromOtherOrchestra(Orchestra *orch)
    {
        if( orch == NULL ) return;
        for(int i=0; i<orch->size(); i++)
        {
            if( !includesInstrument( orch->getInstrViaIndex(i)->getInstrumentID() ) )
                addInstrument( orch->getInstrViaIndex(i) ) ;
        }
    };
        
        
    bool includesOrchestration( Orchestra *orch )
    {
        bool hasOrch = false;
        int index = 0;
        while ( !hasOrch && ( index < orch->size() ) )
        {
            hasOrch = includesInstrument(  orch->getInstrViaIndex(index)->getInstrumentID() );
            index++;
        }
        return hasOrch;
    };
        
        
};
    
class InstrumentLoader
{
protected:
    std::vector<Instrument *> instruments;
public:
    virtual std::vector<Instrument *> load()=0;
};
    
    
class Instruments
{
protected:
    std::vector<Instrument *> instruments;
    InstrumentLoader *loader;
public:
    Instruments(InstrumentLoader * l)
    {
        loader = l;
        instruments = loader->load();
    };
    
    //TODO: write deconstructor
    
    void addInstrument(Instrument * instr)
    {
        instruments.push_back(instr);
    };
    
    virtual Instrument *getInstrument(int id_)
    {
        Instrument *instr =  NULL;
        int index = 0;
        
        while( index < instruments.size() && instr == NULL )
        {
            if( instruments[index]->getInstrumentID() == id_ ) instr = instruments[index];
            index++;
        }
        
        return instr;
    }
    
    std::vector<int> instrumentsWithProfile( boost::shared_ptr<std::vector<int>> profile_ )
    {
        std::vector<int> instr;
        for( int i=0; i<instruments.size(); i++ )
        {
            if( instruments[i]->fitsProfile( *(profile_.get()) ) )
            {
                instr.push_back( instruments[i]->getInstrumentID() );
            }
        }
        return instr;
    };
    
    Orchestra orchestraWithProfile( boost::shared_ptr<std::vector<int>> profile_ )
    {
        Orchestra orch;
        std::vector<int> instr = instrumentsWithProfile(profile_);
        for( int i=0; i<instr.size(); i++ )
        {
            orch.addInstrument( getInstrument(instr[i]));
        }
        return orch;
    };
};
    
    
    
class TangoInstrumentsLoader : public InstrumentLoader
    {
    public:
        
        TangoInstrumentsLoader( std::string instrumentFile_ )
        {
            instrumentFile = instrumentFile_;
        };
        
        
        virtual std::vector<Instrument *> load()
        {
            bool first = true;
            readInstrumentFile.init(instrumentFile); 
            
            while( !readInstrumentFile.eof() )
            {
                std::vector<std::string> tokens = readInstrumentFile.getTokensInLine();
                
                if( first )
                {
                    first = false;
                }
                else
                {
                    int instrID = std::atoi(tokens[0].c_str());
                    std::string n = tokens[1];
                    Instrument *instr = new Instrument(instrID, n);                    
                    instr->setMelTrackID(std::atoi(tokens[2].c_str()));
                    instr->addProfile( std::atoi(tokens[3].c_str()) );
                    
                    instruments.push_back(instr);
                }
            }
            
            return instruments;
            
        };
    protected:
        std::string instrumentFile;
        ReadCSV readInstrumentFile;
    };
    
    class TangoInstruments : public Instruments
    {
    public:
        TangoInstruments() : Instruments( new TangoInstrumentsLoader(INSTRUMENTS_FILE) )
        {
            
        };
        
        
    };
    
    


};
