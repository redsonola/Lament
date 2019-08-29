//
//  MusicPlayer.h
//  InteractiveTangoReadFromAndroid
//
//  Created by courtney on 5/9/15.
//
//

#ifndef InteractiveTangoReadFromAndroid_MusicPlayer_h
#define InteractiveTangoReadFromAndroid_MusicPlayer_h


#endif

namespace CRCPMotionAnalysis {
    
    //generic class containing a soundbank / schema information for a section of music (accompaniment, melody, ornamentation)
    //in charge of selecting which sound to play next, when to play each sound, what other control information to send ableton (if applicable -- not needed yet)
    //TODO: change Max4LIve code to include harmony/sections.
    class MusicSection : public UGEN
    {
    public:
        enum SectionType { MAIN_MELODY=0, ACCOMPANIMENT=1, ORNAMENT=2, COUNTERMELODY=3 };
    protected:
        std::vector<MappingSchema *> mMappingSchemas;
        
//        SoundFileBanks *soundFileBank;
        BeatTiming *beatTimer;
        double beatsPlayed;
//        SoundFile *curSoundFile;
        bool shouldStartFile;
        bool phraseStart;
        Instruments *instrumentsAvailable;
        SectionType sectionType;
    public:
        
        virtual ~MusicSection(){};
        
        Orchestra *getOrchestration()
        {
//            if( curSoundFile != NULL )
//                return curSoundFile->getOrchestration();
//            else return NULL;
            return NULL;
        };
        
        virtual void reset()
        {
//            curSoundFile = NULL;
//            beatsPlayed = 0;
//            shouldStartFile = false;
        }
        
        virtual bool shouldPlayNewFile()
        {
//            if( curSoundFile != NULL )
//            {
//                //            std::cout << "checking if need to play?  beats: " << beatsPlayed << std::endl ;
//                return ( beatsPlayed > curSoundFile->getLength()-1 );
//            }
//            else
//            {
//                std::cout << "curSoundFile is NULL\n";
//                return true;
//            }
        };
        
        virtual void addSchema(MappingSchema *schema)
        {
            mMappingSchemas.push_back(schema);
        }
        
        MusicSection(BeatTiming *timer, Instruments *ins)
        {
            beatTimer = timer;
            instrumentsAvailable = ins;
            
            reset();
        };
        
        virtual int findBusySparse(double window = -1, double secs = -1)
        {
            int bvs = 1;
            //            int found = -1;
            for(int i=0; i<mMappingSchemas.size(); i++)
            {
                int moodNum =  ((PerceptualEvent * )mMappingSchemas[i])->getCurMood(window, secs) ;
                //            std::cout << "in Curfile :"<< ((PerceptualEvent * )mMappingSchemas[i])->getName() << ":  " << moodNum << "     " << seconds << "  \n";
                
                if ( (mMappingSchemas[i]->getMappingType() == MappingSchema::MappingSchemaType::EVENT) &&
                    (!mMappingSchemas[i]->getName().compare("Melody Busy Sparse")  || !mMappingSchemas[i]->getName().compare("Follower Busy Sparse") || !mMappingSchemas[i]->getName().compare("Leader Busy Sparse") || !mMappingSchemas[i]->getName().compare("Couple Busy Sparse")))
                {
                    bvs = moodNum;
                    //                    found = 1;
                }
                //                if(found==-1) std::cout << "NOT FOUND\n";
                //                    else std::cout << "FOUND: " << bvs << std::endl;
            }
            return bvs;
        };
        
        virtual int findBusySparse(int samp)
        {
            PerceptualEvent *ev =  findBusySparseSchema();
            return ev->getCurMood((int)samp);
        }
        
        virtual PerceptualEvent * findBusySparseSchema()
        {
            PerceptualEvent * bvs = NULL;
            //            int found = -1;
            for(int i=0; i<mMappingSchemas.size(); i++)
            {
  
                
                if ( (mMappingSchemas[i]->getMappingType() == MappingSchema::MappingSchemaType::EVENT) &&
                    (!mMappingSchemas[i]->getName().compare("Melody Busy Sparse")  || !mMappingSchemas[i]->getName().compare("Follower Busy Sparse") || !mMappingSchemas[i]->getName().compare("Leader Busy Sparse") || !mMappingSchemas[i]->getName().compare("Couple Busy Sparse")))
                {
                    bvs = (PerceptualEvent *) mMappingSchemas[i];
                    //                    found = 1;
                }
                //                if(found==-1) std::cout << "NOT FOUND\n";
                //                    else std::cout << "FOUND: " << bvs << std::endl;
            }
            return bvs;
        };
        
        std::string findBusySparseName()
        {
            std::string bvs;
            //            int found = -1;
            for(int i=0; i<mMappingSchemas.size(); i++)
            {
                
                if ( (mMappingSchemas[i]->getMappingType() == MappingSchema::MappingSchemaType::EVENT) &&
                    (!mMappingSchemas[i]->getName().compare("Melody Busy Sparse")  || !mMappingSchemas[i]->getName().compare("Follower Busy Sparse") || !mMappingSchemas[i]->getName().compare("Leader Busy Sparse")))
                {
                    bvs = mMappingSchemas[i]->getName();
                    //                    found = 1;
                }
                //                if(found==-1) std::cout << "NOT FOUND\n";
                //                else std::cout << "FOUND: " << bvs << std::endl;
            }
            return bvs;
        };

        
        std::vector<MappingSchema *> getMappingSchemas()
        {
//            std::cout << "mMappingSchemas: " << mMappingSchemas.size(); 
            return mMappingSchemas;
        }
        
        void setInstrumentsAvailable( Instruments *instruments )
        {
            instrumentsAvailable = instruments;
        }
        
        //not yet used
//        virtual SoundFile *getNextStructuredFile(boost::shared_ptr<std::vector<int>> profile, boost::shared_ptr<std::vector<int>> hsprofile, boost::shared_ptr<std::vector<int>> instrumentProfile = NULL)
//        {
//            return NULL;
//
//        };
//
//        virtual SoundFile *getCurFile(boost::shared_ptr<std::vector<int>> hsprofile, float seconds = 0) //if a counter melody, needs to be sync'd with main melody
//        {
//            return NULL;
//        };
        
        virtual void update( float seconds = 0 ){}; //we don't use this form for this... hmm, may refactor.
        
        virtual void update( boost::shared_ptr<std::vector<int>> hsprofile, float seconds = 0 )
        {
            
//            //HERE is where build strangeness happens.
//            if( beatTimer->isOnBeat(0.0, seconds) ) //exactly on beat
//            {
//                //            std::cout <<"on beat? it SHOULD be: " << beatTimer->isOnBeat(0.0, seconds) << ","<< beatTimer->currentBar() << std::endl;
//                beatsPlayed++;
//            }
//            if(shouldPlayNewFile())
//            {
//                shouldStartFile = true;
//                curSoundFile = getCurFile(hsprofile);
//                beatsPlayed = 0;
//            } else shouldStartFile = false;
            
        };
        
        virtual std::vector<ci::osc::Message> getOSC()
        {
//            return getOSC(PLAY_CLIP);
            return getOSC("dummy value woo!");
        };
        
        virtual std::vector<ci::osc::Message> getOSC(std::string addr)
        {
            std::vector<ci::osc::Message> msgs;
            
//            if( shouldStartFile )
//            {
//                //create a message that starts a file -- testing
//                //            boost::shared_ptr<std::vector<int>> profile = curSoundFile->getProfile();
//
//                //for now message - MUST FIX
//                ci::osc::Message msg;
//                msg.setAddress(addr);
//                msg.addIntArg(curSoundFile->getTrackNumber()-1);
//                msg.addIntArg(curSoundFile->getClipNumber()-1);
//                msg.addIntArg(curSoundFile->getAbletonID());
//                msg.addIntArg(sectionType); //melody, accomp., etc.?
//                msg.addIntArg(curSoundFile->getHarmonySectionProfile()->at(0)); //get section of song
//                msg.addIntArg(curSoundFile->getHarmonySectionProfile()->at(1)); //get harmony of song
//
//
//                //            std::cout << "Playing melody: " << curSoundFile->getTrackNumber() << "," << curSoundFile->getClipNumber() << ","<< curSoundFile->getAbletonID() << std::endl;
//                msgs.push_back(msg);
//            }
            
            return msgs;
        };
        
        virtual std::vector<ci::osc::Message>  getOSC(int track, int clip, int AID)
        {
            std::vector<ci::osc::Message> msgs;
            
//            if( shouldStartFile )
//            {
//                //create a message that starts a file -- testing
//                boost::shared_ptr<std::vector<int>> profile = curSoundFile->getProfile();
//
//                //for now message - MUST FIX
//                ci::osc::Message msg;
//                msg.setAddress(PLAY_CLIP);
//                msg.addIntArg(track-1);
//                msg.addIntArg(clip-1);
//                msg.addIntArg(AID);
//                msg.addIntArg(sectionType); //melody, accomp., etc.?
//                msg.addIntArg(curSoundFile->getHarmonySectionProfile()->at(0)); //get section of song
//                msg.addIntArg(curSoundFile->getHarmonySectionProfile()->at(1)); //get harmony of song
//                msgs.push_back(msg);
//            }
            
            return msgs;
        };
    
        BeatTiming *getTimer()
        {
            return beatTimer; 
        }
        
    };
    
    class AccompanimentSection : public MusicSection
    {
    protected:
        Orchestra continuingInstrumentsThroughPhrase; //intrument ids of those that should cont. through the phrase, so there isn't craziness
        int maxInstrumentsToHoldConstantThroughPhrase;
        bool useOrchestrationLimitations;
    public:
        AccompanimentSection(BeatTiming *timer, Instruments *ins) : MusicSection(timer, ins)
        {
            maxInstrumentsToHoldConstantThroughPhrase = 1;
            useOrchestrationLimitations = true;
            sectionType = MusicSection::SectionType::ACCOMPANIMENT;
        };
        
        //this is no longer used, since not using this perceptual measure at the moment
        bool isOrchestration(SignalAnalysisEventOutput *schema)
        {
            OrchThickVsThinEvent* aptr = dynamic_cast<OrchThickVsThinEvent *>(schema);
            return aptr != 0;
        };
        
//        virtual SoundFile *getNextStructuredFile(boost::shared_ptr<std::vector<int>> profile, boost::shared_ptr<std::vector<int>> hsprofile, boost::shared_ptr<std::vector<int>> instrumentProfile)
//        {
//            //first, is there a next file in the storage?
//            StructuredSoundFile *nextFile = NULL;
//
//            if(  curSoundFile !=NULL || !phraseStart )
//            {
//                //            std::cout << "Next File ID: " << curSoundFile->getFileID() << "  instrument:  "<< ((SoundFile *)curSoundFile)->getTrackNumber() <<  "Track, clip, aid" <<curSoundFile->getTrackNumber() << "," << curSoundFile->getClipNumber() << "," << curSoundFile->getAbletonID() << std::endl;
//
//                if(useOrchestrationLimitations)
//                {
//                    //                nextFile = ((StructuredSoundFile *)( (AccompanimentSoundFile *) curSoundFile )->getNextSound(hsprofile, profile, &continuingInstrumentsThroughPhrase, instrumentProfile));
//
//                    nextFile =  (StructuredSoundFile *)soundFileBank->getFileFittingProfileForPhraseStart( profile, hsprofile, instrumentProfile);
//
//                    if( nextFile == NULL )
//                    {
//                        soundFileBank->getFileFittingProfileForPhraseStart(hsprofile, profile, &continuingInstrumentsThroughPhrase, instrumentProfile );
//                    }
//                }
//                else nextFile = ((StructuredSoundFile *)( (StructuredSoundFile *) curSoundFile )->getNextSound(hsprofile, profile));
//            }
//            std::cout << "PVC: " << instrumentProfile->at(0) << " BVS: " << profile->at(0) << std::endl;
//
//
//            //        if( nextFile == NULL && hsprofile->at(1) != 0 )
//            //        {
//            //            SoundFile *cur = curSoundFile;
//            //            std::abort();
//            //        }
//
//            if(nextFile == NULL)
//            {
//
//                //            if(!phraseStart)
//                //            {
//                //                nextFile = (StructuredSoundFile *)soundFileBank->getFileFittingProfile( profile, hsprofile, &continuingInstrumentsThroughPhrase, instrumentProfile);
//                //
//                //                if( nextFile==NULL )
//                //                {
//                //
//                //                    std::cout << "BvS:" << profile->at(0) << ", PvC (instr):" << instrumentProfile->at(0) << ", continuingInstrument: " << continuingInstrumentsThroughPhrase.getInstrViaIndex(0)->getInstrumentID() << " curFileID: " << curSoundFile->getFileID() << "harmony profile:" <<  hsprofile->at(0) << "," << hsprofile->at(1) <<std::endl ;
//                //
//                //                    nextFile = ((StructuredSoundFile *)( (AccompanimentSoundFile *) curSoundFile )->getNextSound(hsprofile, profile, &continuingInstrumentsThroughPhrase, instrumentProfile));
//                //                }
//                ////                std::abort();
//                //            }
//                //
//                //            if(nextFile == NULL)
//                //            {
//                nextFile =  (StructuredSoundFile *)soundFileBank->getFileFittingProfileForPhraseStart( profile, hsprofile, instrumentProfile);
//
//
//                if( nextFile == NULL )
//                {
//                    nextFile = ((StructuredSoundFile *)( (StructuredSoundFile *) curSoundFile )->getNextSound(hsprofile, profile));
//                    std::cout << "Had to do the hack fix\n";
//                }
//                //            }
//            }
//
//            //        if( nextFile!= NULL ){
//            //            std::cout << "Next File ID: " << nextFile->getFileID() << "  instrument:  "<< ((SoundFile *)nextFile)->getTrackNumber() << " nextFileBanks (count):  " << nextFile->numNextFiles() <<  "Track, clip, aid" <<((AccompanimentSoundFile *)nextFile)->getTracks(0) << "," << ((AccompanimentSoundFile *)nextFile)->getClips(0) << "," << ((AccompanimentSoundFile *)nextFile)->getAbletonIDs(0) << std::endl;}
//
//            return (SoundFile *) nextFile;
//
//        };
        
//        virtual SoundFile *getCurFile(boost::shared_ptr<std::vector<int>> hsprofile, float seconds = 0) //if a counter melody, needs to be sync'd with main melody
//        {
//            // build mood profile
//            boost::shared_ptr<std::vector<int>> profile(new std::vector<int>) ;
//            boost::shared_ptr<std::vector<int>> instrumentProfile(new std::vector<int>) ;
//            const float EVENT_MAPPING_WINDOW = 0.75;
//            const float ORCHESTRAL_MAPPING_WINDOW = 2;
//
//
//            for(int i=0; i<mMappingSchemas.size(); i++)
//            {
//                if( !isOrchestration(mMappingSchemas[i]) )
//                {
//                    int moodNum ;
//                    if ( mMappingSchemas[i]->getMappingType() == MappingSchema::MappingSchemaType::EVENT )
//                    {
//                        moodNum = ( (PerceptualEvent * )mMappingSchemas[i] )->getCurMood(EVENT_MAPPING_WINDOW, seconds);
//                        profile->push_back( moodNum );
//                    }
//                    else if ( mMappingSchemas[i]->getMappingType() == MappingSchema::MappingSchemaType::ORCHESTRAL )
//                    {
//                        moodNum = ( (PerceptualEvent * )mMappingSchemas[i] )->getCurMood(ORCHESTRAL_MAPPING_WINDOW, seconds);
//                        instrumentProfile->push_back( moodNum );
//                    }
//                    //                std::cout << "in Curfile :"<< ((PerceptualEvent * )mMappingSchemas[i])->getName() << ":  " << moodNum << "     " << seconds << "  \n";
//                }
//                else
//                {
//                    //note : not even using an orchestral value
//                    profile->push_back(  ( (PerceptualEvent * )mMappingSchemas[i] )->getPhraseLevelValue() );
//                }
//            }
//
//            SoundFile *nextFile = getNextStructuredFile(profile, hsprofile, instrumentProfile);
//
//            //this is effectively an assert since asserts are not working in my Xcode right now !@#$%^&*
//            if(nextFile == NULL)
//            {
//                std::abort();
//            }
//
//            return nextFile;
//        };
        
        //update time cycles at start of phrase
        //TODO: this mapping schema is fucked, man
        virtual void startPhrase(double seconds=0)
        {
            double windowsz = 15; //window of 15 seconds to average over
            for(int i=0; i<mMappingSchemas.size(); i++)
            {
                ( ( PerceptualEvent *) mMappingSchemas[i] )->phrase(windowsz, seconds);
            }
            phraseStart = true;
        }
        
        virtual void addSchema(MappingSchema *schema)
        {
            //have an event that only changes phrase-level (for now)
            if (isOrchestration(schema))
            {
                ( (PerceptualEvent *) schema )->addTimeCycle(MappingTimeCycle::TimeType::PHRASE);
            }
            
            mMappingSchemas.push_back(schema);
        }
        
                //has to be updated with the harmony/section profile
        virtual void update(boost::shared_ptr<std::vector<int>> hsprofile, float seconds = 0 )
        {
//            if( beatTimer->isOnBeat(0.0, seconds) ) //exactly on beat
//            {
//                //            std::cout << "seconds: " << seconds << std::endl;
//                beatsPlayed++;
//            }
//            if(shouldPlayNewFile() && ( hsprofile ) )
//            {
//                shouldStartFile = true;
//                curSoundFile = getCurFile(hsprofile, seconds);
//                assert( hsprofile );
//                assert(curSoundFile != NULL);
//                //            std::cout << "accompaniment sound ID: " << curSoundFile->getFileID()  << std::endl;
//                beatsPlayed = 0;
//
//                //switch the constant instrument
//                if( phraseStart )
//                {
//                    phraseStart = false;
//
//                    continuingInstrumentsThroughPhrase.clear();
//                    Orchestra *curOrch = curSoundFile->getOrchestration();
////                    assert( !curOrch->empty() );
//                    for(int i=0; i<maxInstrumentsToHoldConstantThroughPhrase && i<curOrch->size(); i++)
//                    {
//                        continuingInstrumentsThroughPhrase.addInstrument(curOrch->getInstrViaIndex(i));
//                    }
//                }
//
//            } else shouldStartFile = false;
        };
        
        virtual std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            
            
//            if( shouldStartFile )
//            {
//                assert( curSoundFile != NULL );
//                //                        std::cout << "Accompaniment Play now: " << curSoundFile->getTrackNumber() << ",  " << curSoundFile->getClipNumber() << std:: endl ;
//            }
//            else return msgs;
//
//            AccompanimentSoundFile *afile = (AccompanimentSoundFile *) curSoundFile;
//            if( ( curSoundFile==NULL ) && shouldStartFile )
//                std::cout << "is NULL\n";
//
//            for( int i=0; i<afile->getClipSize(); i++)
//            {
//                std::vector<ci::osc::Message> mmsgs = MusicSection::getOSC(afile->getTracks(i), afile->getClips(i), afile->getAbletonIDs(i));
//                //            std::cout << "Accompaniment msg play: " <<afile->getTracks(i) << ","<< afile->getClips(i)<<"," << afile->getAbletonIDs(i) << std:: endl ;
//
//                if ( mmsgs.size() > 0 ) msgs.push_back(mmsgs[0]);
//            }
            
            return msgs;
        };
        
    };
    
    class MelodySection : public MusicSection
    {
    protected:
        FootOnset *fo;
        double secondsPlayed, secondsStarted; //melodies measure in seconds, since last shorter than accompaniment measures
        float  mappingPerceptionWindowForMelody ; //maybe change the placement of this param????
        
        bool lastStepped;
        
        float curSeconds;
    public:
        MelodySection( BeatTiming *timer, FootOnset *onset, Instruments *ins, float perWindowSize=5 ) : MusicSection( timer, ins )
        {
            fo = onset;
            secondsPlayed = 0;
            secondsStarted = 0;
            
            //TODO: check
            mappingPerceptionWindowForMelody = 0.75;// perWindowSize; //default is 0.75 second...
            
            sectionType = MusicSection::SectionType::COUNTERMELODY;
            lastStepped = false;
        };
        
        
        
//        virtual SoundFile *getCurFile(boost::shared_ptr<std::vector<int>> hsprofile, float seconds = 0) //if a counter melody, needs to be sync'd with main melody
//        {
//            // build mood profile & also moodprofile for instrument choice, if relevant
//            boost::shared_ptr<std::vector<int>> profile(new std::vector<int>);
//            boost::shared_ptr<std::vector<int>> instrumentProfile(new std::vector<int>);
//
//            //TODO: added busy v sparse
//            //        profile->push_back(1);
//            for(int i=0; i<mMappingSchemas.size(); i++)
//            {
//                int moodNum =  ((PerceptualEvent * )mMappingSchemas[i])->getCurMood(mappingPerceptionWindowForMelody, seconds) ;
//                //            std::cout << "in Curfile :"<< ((PerceptualEvent * )mMappingSchemas[i])->getName() << ":  " << moodNum << "     " << seconds << "  \n";
//
//                if ( mMappingSchemas[i]->getMappingType() == MappingSchema::MappingSchemaType::EVENT )
//                    profile->push_back( moodNum );
//                else if(( mMappingSchemas[i]->getMappingType() == MappingSchema::MappingSchemaType::ORCHESTRAL )  )
//                    instrumentProfile->push_back( moodNum );
//            }
//
//
//            return getNextStructuredFile(profile, hsprofile, instrumentProfile);
//        };
        
        virtual bool stepped(){ return lastStepped; };
        
        virtual void update(boost::shared_ptr<std::vector<int>> hsprofile, float seconds = 0)
        {
//            secondsPlayed = beatTimer->getTimeInSeconds() - secondsStarted; //how long in seconds has the file been playing?
//            curSeconds = seconds;
//
//            if( beatTimer->isOnBeat(0.0, seconds) ) //exactly on beat
//            {
//                beatsPlayed++;
//            }
//
//            lastStepped = fo->isStepping() ;
//
//            if(shouldPlayNewFile() && fo->isStepping() ) //wait for a foot onset to start
//            {
//                shouldStartFile = true;
//                curSoundFile = getCurFile(hsprofile, seconds);
//                beatsPlayed = 0;
//                secondsStarted = beatTimer->getTimeInSeconds();
//            } else shouldStartFile = false;
//
//            if(curSoundFile != NULL)
//            {
//                ( (MelodySoundFile *) curSoundFile)->changeHarmonyIfHalfway(secondsPlayed);
//            }
        };
        
        virtual int findBusySparse(double window = -1, double secs = -1)
        {
            if(window==-1) window = mappingPerceptionWindowForMelody;
            if(secs ==-1) secs = curSeconds;
            return MusicSection::findBusySparse(mappingPerceptionWindowForMelody, window);
        }
        
        virtual boost::shared_ptr<std::vector<int>> getHarmonySectionProfile()
        {
//            if ( curSoundFile != NULL )
//                return curSoundFile->getHarmonySectionProfile();
//            else
//            {
//                return boost::shared_ptr<std::vector<int>>(NULL);
//            }
            return boost::shared_ptr<std::vector<int>>(NULL);

        };
        
        virtual std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;// = MusicSection::getOSC(PLAY_MELODY_CLIP);
//            if(_COUT_DEBUG_)
//                if(msgs.size() > 0)
//                    std::cout << msgs[0].getAddress() << ","  << msgs[0].getArgAsInt32(0) << "," << msgs[0].getArgAsInt32(1) << "," << msgs[0].getArgAsInt32(2) << std::endl;
            return msgs;
            
        };
        
        
//        //REFACTOR -- THIS IS A HACK FOR NOW!!!!
//        virtual bool shouldPlayNewFile()
//        {
//            if( curSoundFile != NULL )
//            {
//                int bvs = findBusySparse();
//                if( bvs ==1 )
//                    return ( secondsPlayed > curSoundFile->getLengthSeconds() );
//                else if( bvs==2 )
//                    return ( secondsPlayed > curSoundFile->getLengthSeconds() * 3.0);
//                else
//                    return ( secondsPlayed > curSoundFile->getLengthSeconds() * 6.0);
//
//            }
//            else return true;
//        };
//    };
    
    
    //could have a different ornament section for each gesture....
    //for circling, turn on looping & disable at end
    //this base class is used for circling...
//    class OrnamentationSection : public MelodySection
//    {
//    protected:
////        GestureToMusic *gesture;
//        bool justStartedPlaying;
//        Orchestra curOrch;
//
//        // build mood profile
//        std::vector<int> continuousInstrumentProfile;
//    public:
////        OrnamentationSection(GestureToMusic *g, BeatTiming *timer, FootOnset *onset, Instruments *ins, float perWindowSize=5 ) : MelodySection( timer, onset, ins )
////        {
////            gesture = g;
////            sectionType = MusicSection::SectionType::ORNAMENT;
////            justStartedPlaying = false;
////
////            continuousInstrumentProfile.push_back(1);
////        };
//
//        virtual bool shouldPlayNewFile()
//        {
//            bool should =  gesture->shouldSendPlay() ;
//            justStartedPlaying = should;
//            if( curSoundFile != NULL )
//                should = should || ( gesture->stillPlaying() && ( beatsPlayed > curSoundFile->getLength()-1 ) ) ;
//            return should;
//        };
//
//        virtual SoundFile *getCurFile(boost::shared_ptr<std::vector<int>> hsprofile, Orchestra *orchestration, float seconds = 0) //if a counter melody, needs to be sync'd with main melody
//        {
//            // build mood profile
//            boost::shared_ptr<std::vector<int>> profile(new std::vector<int>);
//
//            //TODO: added busy v sparse
//            //        profile->push_back(1);
//            for(int i=0; i<mMappingSchemas.size(); i++)
//            {
//                int moodNum =  ((PerceptualEvent * )mMappingSchemas[i])->getCurMood(mappingPerceptionWindowForMelody, seconds) ;
//                profile->push_back( moodNum );
//            }
//
//
//            return getNextStructuredFile(profile, hsprofile, orchestration);
//        };
//
//        //this changes ornament so it does not switch instruments with the melody, if it goes on longer than melody -- keeps same orchestration while still playing
//        void determineOrchestration(Orchestra *orchestration)
//        {
//            //only switch orchestration if just started playing ornament or if null
//            if(justStartedPlaying || curOrch.size() <= 0)
//            {
//                curOrch.clear();
//
//                //check if it is a continuous instrument
//                //it must now be continous for circling...
//
//                boost::shared_ptr<std::vector<int>> profile(new std::vector<int>);
//                for(int i=0; i<continuousInstrumentProfile.size(); i++)
//                    profile->push_back(continuousInstrumentProfile[i]);
//
//                if( orchestration->fitsProfile(profile))
//                {
//                    curOrch.addfromOtherOrchestra(orchestration);
//                }
//                else
//                {
//                    //add a random continuous instrument
//                    std::vector<int> ids =  instrumentsAvailable->instrumentsWithProfile(profile);
//                    int instrid = ids[ rand() % ids.size() ];
//                    curOrch.addInstrument(instrumentsAvailable->getInstrument(instrid));
//                }
//                profile.reset();
//            }
//        };
//
//        virtual void update(boost::shared_ptr<std::vector<int>> hsprofile, Orchestra *orchestration, float seconds = 0)
//        {
//
//            secondsPlayed = beatTimer->getTimeInSeconds() - secondsStarted; //how long in seconds has the file been playing?
//
//            if( beatTimer->isOnBeat(0.1, seconds) ) //exactly on beat
//            {
//                beatsPlayed++;
//            }
//
//            if( shouldPlayNewFile() ) //wait for a foot onset to start
//            {
//                determineOrchestration(orchestration);
//                shouldStartFile = true;
//                curSoundFile = getCurFile(hsprofile, &curOrch, seconds);
//                beatsPlayed = 0;
//                secondsStarted = beatTimer->getTimeInSeconds();
//                if( curSoundFile == NULL )
//                {
//                    boost::shared_ptr<std::vector<int>> profile(new std::vector<int>);
//                    curSoundFile = getNextStructuredFile(profile, hsprofile, orchestration); //??
//                }
//
//            } else shouldStartFile = false;
//
//        };
//
//
//
//        virtual SoundFile *getNextStructuredFile(boost::shared_ptr<std::vector<int>> profile, boost::shared_ptr<std::vector<int>> hsprofile, Orchestra *orchestration)
//        {
//
//            //like the accompaniment, this only chooses based on harmony, not on previous file
//            SoundFile *nextFile =  soundFileBank->getFileFittingProfile( profile, hsprofile, orchestration );
//            return (SoundFile *) nextFile;
//        };
//
//        std::vector<ci::osc::Message> getOSC()
//        {
//            std::vector<ci::osc::Message> msgs;
//
//
//            if( shouldStartFile &&  ( curSoundFile != NULL ) ) //ok so it will not torpedo everything.....
//            {
//                //            std::cout << "This should be sending messages to circle.\n";
//
//                std::vector<ci::osc::Message> gmsgs = gesture->getOSC();
//
//                AccompanimentSoundFile *afile = (AccompanimentSoundFile *) curSoundFile;
//                if( ( curSoundFile==NULL ) && shouldStartFile )
//                    std::cout << "is NULL\n";
//
//                //this needs to be refactored yikes
//                for( int i=0; i<afile->getClipSize(); i++)
//                {
//                    std::vector<ci::osc::Message> mmsgs = MusicSection::getOSC(afile->getTracks(i), afile->getClips(i), afile->getAbletonIDs(i));
//                    if ( mmsgs.size() > 0 )
//                    {
//                        mmsgs[0].setAddress(PLAY_CIRCLING_CLIP); //set a different message so it gets sent somewhere different
//                        msgs.push_back(mmsgs[0]);
//                    }
//                    //                std::cout << "Circling: " << mmsgs[0].getAddress() << afile->getTracks(i) << ","<<afile->getClips(i) << std::endl;
//                }
//
//            }
//
//            std::vector<ci::osc::Message> gmsgs = gesture->getOSC();
//            for(int i=0; i<gmsgs.size(); i++)
//            {
//                msgs.push_back(gmsgs[i]);
//                //            if( !gmsgs[i].getAddress().compare(CIRCLING_VOLUME) )
//                //            {
//                //                std::cout << "In Player: " << CIRCLING_VOLUME << gmsgs[i].getArgAsFloat(0) << std::endl;
//                //            }
//            }
//
//            return msgs;
//        };
//    };
//
//    //this ornament is different, since it does reference pre-written bits of melody, but instead triggers a melody generation based on current main melody
//    class BoleoOrnamentationSection : public OrnamentationSection
//    {
//    protected:
//        Orchestra curOrch;
//        bool startOrnament;
//        std::vector<int> profile;
//    public:
//        BoleoOrnamentationSection(GestureToMusic *g, BeatTiming *timer, FootOnset *onset, Instruments *ins, float perWindowSize=5 ) : OrnamentationSection(g, timer, onset, ins, perWindowSize)
//        {
//        };
//
//        virtual void update(boost::shared_ptr<std::vector<int>> hsprofile, Orchestra *orchestration, float seconds = 0)
//        {
//            curOrch.clear();
//            curOrch.addfromOtherOrchestra(orchestration);
//
//            startOrnament = gesture->shouldSendPlay() ;
//
//            profile.clear();
//            for(int i=0; i<mMappingSchemas.size(); i++)
//            {
//                profile.push_back( ((PerceptualEvent * )mMappingSchemas[0])->getCurMood(mappingPerceptionWindowForMelody, seconds) );
//            }
//
//        };
//
//        ///send --> boleo_message which_track busy_sparse
//        std::vector<ci::osc::Message> getOSC()
//        {
//            assert( mMappingSchemas.size() >=1 );
//
//            std::vector<ci::osc::Message> msgs;
//            std::vector<ci::osc::Message> gmsgs = gesture->getOSC();
//
//            if( startOrnament && curOrch.size() > 0 ) //doesn't start until main melody starts
//            {
//                ci::osc::Message msg;
//                msg.setAddress(CREATE_BOLEO_ORNAMENT);
//
//                //--- THIS IS FOR DEBUGGING ERASE LATER
//
//                //            std::cout << CREATE_BOLEO_ORNAMENT << "," << 1699 << std::endl;
//                std::cout << CREATE_BOLEO_ORNAMENT << "," << curOrch.getInstrViaIndex(0)->getTrackID() <<
//                "," << profile[0] << std::endl;
//
//                msg.addIntArg( curOrch.getInstrViaIndex(0)->getTrackID() ); //this orch is from main melody, which should have 1 instrument only --> this sends the correct track to ableton
//
//                for(int moodNum : profile)
//                    msg.addIntArg(moodNum);
//
//                msgs.push_back(msg);
//
//                for(int i=0; i<gmsgs.size(); i++)
//                {
//                    msgs.push_back(gmsgs[i]);
//                }
//            }
//
//            return msgs;
//        };
    };

    
    //MAIN melody controls song structure! Everything is based off the melody
    class MainMelodySection : public MelodySection
    {
    protected:
        std::vector<int> song_structure;
        int where_in_song_structure;
        
        //TODO: collapse these... yup. --Courtney
        bool phraseEndNext;
        bool phraseEndWaiting;
        bool endOfSong;
        
        //this variables are to determine whether it has been more than 4 bars since main melody has played.. if so, let music player know to pause all the sections
        int numberOfBarsAfterStepBeforePausing;
        bool paused;
        long lastBarStepped;
        
    public:
        MainMelodySection(BeatTiming *timer, FootOnset *onset, Instruments *ins, float perWindowSize=1) : MelodySection(timer, onset, ins, perWindowSize)
        {
            where_in_song_structure = 0;
            reset();
            sectionType = MusicSection::SectionType::MAIN_MELODY;
        };
        
        virtual void reset()
        {
            //TODO: everhything about this is terrible -- collapse this, sorry, coding world......
            phraseStart = false; //when phrase starts, after foot onset
            phraseEndNext = false; //last phrase bit, AT foot onset of last phrase
            phraseEndWaiting = false; //before start of next phrase and after start of next.
            where_in_song_structure = 0;
//            curSoundFile = NULL;
            endOfSong = false;
            
            numberOfBarsAfterStepBeforePausing = 2;
            paused = false;
            lastBarStepped = 0;
        }
        
        virtual void updatePause()
        {
            if( stepped() )
            {
                lastBarStepped = beatTimer->currentBar();
                paused = false;
            }
            else
            {
                paused = beatTimer->currentBar() - lastBarStepped >= numberOfBarsAfterStepBeforePausing;
            }
        };
        
        virtual bool shouldPause(){ return paused; };
        
        virtual void timesToRepeatSection()=0;
        
        virtual  void setSongStructure(std::vector<int> sections)
        {
            song_structure = sections;
        };
        
        inline bool afterEndOfPhrase(){ return phraseEndWaiting; };
        inline bool isStartOfPhrase(){ return phraseStart; };
        
        virtual void update(boost::shared_ptr<std::vector<int>> hsprofile, float seconds = 0)
        {
//            phraseStart = false;
//            MelodySection::update(hsprofile, seconds);
//            updatePause();
        };
        
//        virtual SoundFile *getNextStructuredFile(boost::shared_ptr<std::vector<int>> profile, boost::shared_ptr<std::vector<int>> hsprofile, boost::shared_ptr<std::vector<int>> instrumentProfile)
//        {
//            phraseEndNext = false;
//            phraseEndWaiting = false;
//
//            //first, is there a next file in the storage?
//            StructuredSoundFile *nextFile = NULL;
//            if( curSoundFile == NULL )  //doesn't recognize curSoundFile != NULL......
//            {
//                std::cout << "curSoundFile is null\n";
//            }
//            else
//            {
//                nextFile = ((StructuredSoundFile *)( (MelodySoundFile *) curSoundFile )->getNextSound(profile, hsprofile));
//            }
//
//            if(nextFile == NULL)
//            {
//                phraseStart = true; //we are starting a phrase
//                //            std::cout << "starting a pharse" << std::endl;
//
//                //starting a new melodic phrase, so harmony is #1 & which melodic phrase is determined by structure.
//                hsprofile->push_back(song_structure[where_in_song_structure]);
//                hsprofile->push_back(1);
//
//                where_in_song_structure++; //now increment where we are in the overall song structure
//                if(where_in_song_structure >= song_structure.size()) where_in_song_structure = 0; //cycle back to zero
//
//                nextFile = ( ( StructuredSoundFile * ) soundFileBank->getFileFittingProfileForPhraseStart( profile, hsprofile, instrumentProfile ) );
//
//            }
//            else
//            {
//                //check if ending phrase
//                phraseEndNext = ( (StructuredSoundFile *)nextFile )->endOfPhrase();
//            }
//
//            if( nextFile == NULL )
//            {
//                assert( nextFile != NULL);
//                std::cerr << "SOUND FILE IS NULL\n";
//                std::abort();
//            }
//
//            //        if( nextFile!= NULL ){
//            //            std::cout << "Next File ID: " << nextFile->getFileID() << "  instrument:  "<< ((SoundFile *)nextFile)->getTrackNumber() << " nextFileBanks (count):  " << nextFile->numNextFiles() << std::endl;}
//            //
//            //        if( nextFile->numNextFiles() == 2 )
//            //        {
//            //            std::abort();
//            //        }
//
//            return (SoundFile *) nextFile;
//
//        };
//
        virtual void addSchema(MappingSchema *schema)
        {
            mMappingSchemas.push_back(schema);
        };
        
//        virtual bool shouldPlayNewFile() //checks for waiting at the end of a phrase
//        {
//            if( curSoundFile != NULL )
//            {
//                bool ready = MelodySection::shouldPlayNewFile() ;
//                phraseEndWaiting = ready && phraseEndNext; //end of phrase and waiting for the next footfall.
//                return ready;
//            }
//            else return true;
//        };
        
    };
    
    //TODO: TRY... length btw steps controlling reverb tails.... could be interesting!!!
    
    class MusicPlayer : public UGEN //should have a song structure, will see
    {
    protected:
        MainMelodySection *main_melody; //determines structure (for now)
        std::vector<MelodySection *> c_melodies;
        std::vector<AccompanimentSection *> accompaniments;
//        std::vector<OrnamentationSection *> ornaments;
        
        boost::shared_ptr< std::vector<int> > curHarmonyProfile; //always have a harmony profile
        
    public:
        MusicPlayer() : UGEN()
        {
            curHarmonyProfile.reset();
            main_melody=NULL;
        };
        
        //go back to beginning -- clean slate
        virtual void reset()
        {
            if ( main_melody != NULL ) main_melody->reset();
            curHarmonyProfile.reset();
            for(int i=0; i<accompaniments.size(); i++)
            {
                accompaniments[i]->reset();
            }
//            for(int i=0; i<c_melodies.size(); i++)
//            {
//                c_melodies[i]->reset();
//            }
//            for(int i=0; i<ornaments.size(); i++)
//            {
//                ornaments[i]->reset();
//            }
        };
        
        
        virtual void update(float seconds = 0)
        {
            if(main_melody == NULL) return; 
            
            boost::shared_ptr<std::vector<int>> hsprofile(new std::vector<int> ); //empty one for main melody... should fix that setup
            Orchestra curMelodyOrchestration;
            
            main_melody->update(hsprofile, seconds);
            hsprofile.reset();
            hsprofile = main_melody->getHarmonySectionProfile();
            curMelodyOrchestration.addfromOtherOrchestra( main_melody->getOrchestration() );
            
            //use last harmony profile for melody that is not playing, etc., if not first one
            if( hsprofile.get() == NULL ) hsprofile = curHarmonyProfile;
            
            //for now
            if( hsprofile.get() != NULL )
            {
                
                if( !main_melody->shouldPause() )
                {
                    for(int i=0; i<c_melodies.size(); i++)
                    {
                        c_melodies[i]->update(hsprofile, seconds);
                        curMelodyOrchestration.addfromOtherOrchestra( c_melodies[i]->getOrchestration() );
                    }
                    
                    for(int i=0; i<accompaniments.size(); i++)
                    {
                        if( main_melody->isStartOfPhrase() )
                            accompaniments[i]->startPhrase();
                        accompaniments[i]->update(hsprofile, seconds);
                    }
                }
//                //TODO: OK FIX SOON!!!!!
//                for(int i=0; i<ornaments.size(); i++)
//                {
//                    ornaments[i]->update(hsprofile, &curMelodyOrchestration, seconds);
//                }
            }
            curHarmonyProfile.reset();        
            curHarmonyProfile = hsprofile;
        };
        
        virtual std::vector<ci::osc::Message> getOSC()
        {
            //collect all the OSC messages to send
            std::vector<ci::osc::Message> msgs;
            std::vector<ci::osc::Message> cmsgs;
            std::vector<ci::osc::Message> amsgs;
//            std::vector<ci::osc::Message> omsgs;
            
            if( main_melody == NULL ) return msgs;
            
            msgs = main_melody->getOSC();
            //        for(int i=0; i<msgs.size(); i++)
            //        {
            //            std::cout << "track: " << msgs[i].getAddress() << std::endl;
            //        }
            
            for(int i=0; i<c_melodies.size(); i++)
            {
                cmsgs = c_melodies[i]->getOSC();
                for(int i=0; i<cmsgs.size(); i++)
                    msgs.push_back(cmsgs[i]);
            }
            
            if( !main_melody->afterEndOfPhrase() && !main_melody->shouldPause() ) // wait for end of phrase -- maybe send an osc to stop ableton transport
            {
                for(int i=0; i<accompaniments.size(); i++)
                {
                    amsgs =  accompaniments[i]->getOSC();
                    for(int i=0; i<amsgs.size(); i++)
                    {
                        msgs.push_back(amsgs[i]);
                        //                    std::cout << amsgs[i].getAddress());
                    }
                }
            }
            
//            for(int i=0; i<ornaments.size(); i++)
//            {
//                omsgs = ornaments[i]->getOSC();
//                for(int i=0; i<omsgs.size(); i++)
//                    msgs.push_back(omsgs[i]);
//            }
            
            return msgs;
        };
        
        virtual void setMainMelody(MainMelodySection *melody)
        {
            main_melody = melody;
        };
        virtual void addCounterMelody(MelodySection *cMelody)
        {
            c_melodies.push_back(cMelody);
        };
        virtual void clearCurAccompaniment()
        {
            accompaniments.clear();
        };
//        virtual void clearCurOrnaments()
//        {
//            ornaments.clear();
//        };
        virtual void addAccompaniment(AccompanimentSection *accomp)
        {
            accompaniments.push_back(accomp);
        };
//        virtual void addOrnament(OrnamentationSection *ornament)
//        {
//            ornaments.push_back(ornament);
//        };
    };
    
};

