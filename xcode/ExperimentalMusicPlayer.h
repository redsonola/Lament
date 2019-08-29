//
//  ExperimentalMusicPlayer.h
//  MagneticGardel
//
//  Created by courtney on 11/17/17.
//
//

#ifndef ExperimentalMusicPlayer_h
#define ExperimentalMusicPlayer_h

#include "sequence_player.h"

#define MAX_DEAD_PLAYERS 200
#define HOW_LONG_TO_STAY_STILL_FOR_CADENCE_WINDOW_SECONDS 2
#define PERCENTAGE_RANGE_COUNTS_AS_STILL 0.18 //of busy sparse scale

namespace CRCPMotionAnalysis
{
    
    class GeneratedMelodySection : public MainMelodySection
    {
    protected:
        std::vector<MelodyGenerator *> generator;
        BusyVsSparseEvent *bsCouple; //need for section changes
        GeneratedMelodySection *sectionDecisionMaker;
        
        //std::vector<int> song_structure; -- inherited
        //int where_in_song_structure; -- inherited
        //since we don't have structure-defined phrases, we have min. times
        std::vector<float> songStructureDurations;
        std::vector<int> expInstrumentsforSections; //this will be hardcoded for this performance -- TODO: fix
        int sectionGeneratorIndex;
        float lastSectionChange;
        
        float lastTimePlayed;
        
        int whichDancer;
        
        std::vector<ci::osc::Message> melodyMessages; 
    public:
        
        GeneratedMelodySection (BeatTiming *timer, FootOnset *onset, std::vector<MelodyGenerator *> gen, int whichDancer_, Instruments *ins=NULL, float perWindowSize=1) : MainMelodySection(timer, onset, ins, perWindowSize)
        {
            generator = gen;
            timesToRepeatSection();
            sectionGeneratorIndex = 0;
            lastSectionChange = timer->getTimeInSeconds();
            bsCouple = NULL;
            sectionDecisionMaker = NULL;
            
            whichDancer = whichDancer_;
            where_in_song_structure =0;
            
            std::cout << "Number of current generators: " << generator.size() << std::endl;
            changeInstrument();
            sendSection();

        };
        
        virtual void update(boost::shared_ptr<std::vector<int>> hsprofile, float seconds = 0)
        {
            changeSectionIfNeeded(seconds);
            if( fo->isStepping()){
                update(seconds);
            }
        }
        
        void setCoupleBS(BusyVsSparseEvent *bs)
        {
            bsCouple  = bs;
        }
        
        void setExpInstrumentsforSections(std::vector<int> ins)
        {
            expInstrumentsforSections = ins;
        }
        
        void resetSong()
        {
            where_in_song_structure = 0;
            lastSectionChange = beatTimer->getTimeInSeconds();
            sectionGeneratorIndex = song_structure[where_in_song_structure] - 1;
            std::cout << "changing section (reset): " << where_in_song_structure << " instr: " << expInstrumentsforSections[where_in_song_structure] << std::endl;
            changeInstrument();
            sendSection();
        }
        
        void changeSectionIfNeeded(float seconds)
        {
            if(bsCouple != NULL)  // changing sections actively
            {

                if( seconds - lastSectionChange >= songStructureDurations[where_in_song_structure] )
                {
                    if( bsCouple->getCurMood(HOW_LONG_TO_STAY_STILL_FOR_CADENCE_WINDOW_SECONDS, seconds) <= bsCouple->getMaxMood() * PERCENTAGE_RANGE_COUNTS_AS_STILL)
                    {
                        where_in_song_structure++;
                        if(where_in_song_structure >= song_structure.size())
                            where_in_song_structure = 0; //OR, end;
                    
                        sectionGeneratorIndex = song_structure[where_in_song_structure] - 1;
                        std::cout << "changing section: " << where_in_song_structure << " instr: " << expInstrumentsforSections[where_in_song_structure] << std::endl;
                        lastSectionChange = seconds;
                        sendSection();
                    }
                }
            }
            else if(sectionDecisionMaker != NULL) //getting section changes from somewhere else ------- TODO: FIX SINCE HAVE ANOTHER STRUCT FOR THIS in other tango system
            {
                int where = sectionDecisionMaker->getWhereInSong();
                sectionGeneratorIndex = song_structure[where] - 1;
                where_in_song_structure = where;
                sendSection();

            }
            changeInstrument();
//            std::cout << "melody section: " << sectionGeneratorIndex+1 << endl;

        }
        
        void changeInstrument()
        {
            if(where_in_song_structure < expInstrumentsforSections.size())
            {
                ci::osc::Message msg;
                msg.setAddress(EXPMUSIC_MELODY_INSTRUMENT);
                msg.append(whichDancer);
                msg.append(expInstrumentsforSections[where_in_song_structure]);
                melodyMessages.push_back(msg);
//                std::cout << "instrument;"<< whichDancer<< "," << expInstrumentsforSections[where_in_song_structure] << endl;
            }
            
        }
        
        void sendSection()
        {
            //sends where in song structure... not really sectiion...  to max or ableton
            ci::osc::Message msg;
            msg.setAddress(EXPMUSIC_SECTION);
            msg.append(where_in_song_structure+1);
            melodyMessages.push_back(msg);
        }
        
        void setMelodySectionDecider(GeneratedMelodySection *mel)
        {
            sectionDecisionMaker = mel;
        }
        
        int getWhereInSong()
        {
            return where_in_song_structure;
        }
        
        int getSection()
        {
            return song_structure[where_in_song_structure];
        }
        
        virtual void update(float seconds = 0)
        {
            curSeconds = seconds;
            changeSectionIfNeeded(seconds);
            sendSection(); //for now

//
            //NOTE TO SELF -- UNCOMMENT THIS AFTTER RECORDING!!!!
//            if(where_in_song_structure < expInstrumentsforSections.size())
//            {
//                ci::osc::Message msg;
//                msg.setAddress(EXPMUSIC_MELODY_INSTRUMENT);
//                msg.addIntArg(whichDancer);
//                msg.addIntArg(expInstrumentsforSections[where_in_song_structure]);
//                melodyMessages.push_back(msg);
//            }
            
            //if busy/sparse = 1 then wait a bit (at least an eighth note) before re-genning
            float timeDiff = seconds - lastTimePlayed;
            float quarter = (1.0f / ((float) generator[0]->getBPM() /  60.0f)) ; //fix
            float eight = quarter / 2.0f;
            float sixteenth = eight / 2.0f;

            int bsorig = std::round( MusicSection::findBusySparse((int)20) );
            PerceptualEvent *ev = findBusySparseSchema();

            std::vector<double> cutoffs = { 0.3, 0.55, 0.75, 0.85, 0.95 };
            int bs = std::round(ev->getNonLinearScalingbyFiat(cutoffs, 5, bsorig));
            
//            std::cout << "Bs before:" <<bsorig<<" BS now: " << bs << " min:" << ev->getMinMood() << " max: "<< ev->getMaxMood() << endl;

            if( timeDiff < sixteenth ) return; // no faster than sixteenth notes but don't place such a hard limit on density... see. 
            
            if(generator[sectionGeneratorIndex]->oneToOne() && fo->isStepping() ) //wait for a foot onset to start
            {
                //ok now generate a melody
                generator[sectionGeneratorIndex]->update(seconds);
            } //else implement a non-one to one mode solution
            else if( fo->isStepping())
            {
                generator[sectionGeneratorIndex]->update(bs, seconds);
//                std::cout << generator[sectionGeneratorIndex]->getFile();
                lastTimePlayed = seconds; //beatTimer->getTimeInSeconds();
            }
            
        };
        
        
        std::vector<MidiNote> getBufferedNotes()
        {
            return generator[sectionGeneratorIndex]->getCurNotes();
        }
        
         virtual std::vector<ci::osc::Message> getOSC()
        {
            //probably do nothing here... we'll see
            std::vector<ci::osc::Message> msgs = MainMelodySection::getOSC(); //this should return nothing so far.
            
            for(int i =0; i<melodyMessages.size(); i++)
                msgs.push_back(melodyMessages[i]);
            
            melodyMessages.clear();
            
            return msgs;
        }
        
        virtual void timesToRepeatSection()
        {
            //2 sections --  hard-coded -- dear god I will refactor this after the deaadline
            song_structure.clear();
            song_structure.push_back(1);
            songStructureDurations.push_back(30);
            song_structure.push_back(3);
            songStructureDurations.push_back(30);
            song_structure.push_back(2);
            songStructureDurations.push_back(30);
            song_structure.push_back(3);
            songStructureDurations.push_back(30);
            song_structure.push_back(1);
            songStructureDurations.push_back(30);
        }
        
        double getTicksPerBeat()
        {
            return generator[sectionGeneratorIndex]->getTicksPerBeat(0);
        }
        
    };
    
    class GeneratedAccompanmentSection : public AccompanimentSection
    {
    protected:
        std::vector<ChordGeneration *> generators;
        std::vector<std::vector<ChordGeneration *>> generatorsEaSection;
        std::vector<std::vector<int>> expInstrumentsforSections; //this will be hardcoded for this performance -- TODO: fix

        GeneratedMelodySection *sectionDecisionMaker;
        
        std::vector<std::vector<MidiNote>> notes;
        int BEATSPERMEASURE;
        
        //responds to busy sparse by doing the opposite or doing the parallel
        int MEASURES_TO_FLIP_BVS_MIRROR_OR_COUNTER;
        bool bvsMirror;
        bool percBvsMirror;
        bool percBvsChanged;
        int measureBVSCount;
        int bvs;
        int percbvs;  //sends to max for percussion values
        bool pauseForFillOrClose;
        
        bool fiveStopped;
        bool bvsChangedTo2;
        
        int curGen;
        
        std::vector<ci::osc::Message> harmonyMessages;
        int curSection;
        int sampleplay;
        
        int where_in_song_structure;

        
    public:
        
        GeneratedAccompanmentSection(BeatTiming *timer, Instruments *ins) : AccompanimentSection(timer, ins)
        {
            BEATSPERMEASURE = 4;
            curGen = 0;
            
            MEASURES_TO_FLIP_BVS_MIRROR_OR_COUNTER = 8;
            measureBVSCount = 0;
            
            bvsMirror = true;
            percBvsMirror = true;
            percBvsChanged = true; //only send OSC to Max 7 if this is aa different value
            percbvs = -1;
            
            pauseForFillOrClose = false;
            fiveStopped = false;
            bvsChangedTo2 = false;
            
            generatorsEaSection.push_back(std::vector<ChordGeneration *>());
            generatorsEaSection[0].push_back(new ChordGeneration());
            generatorsEaSection[0].push_back(new ChordGenerationPizzInspired());
            generators = generatorsEaSection[0];// 1st section chords
            
            //section 2 is a variation of section 1 kind oof...
            generatorsEaSection.push_back(std::vector<ChordGeneration *>());
            generatorsEaSection[1].push_back(new ChordGeneration());
            generatorsEaSection[1].push_back(new ChordGenerationPizzInspired());
            
            //section 3 is in the contrasting key... 
            generatorsEaSection.push_back(std::vector<ChordGeneration *>());
            generatorsEaSection[2].push_back(new ChordGenerationSection2());
            
            defaultExpInstrumentsforSections();
            
            sampleplay = 0;
        }
        
        void setBVSMirroring()
        {

            if(measureBVSCount >= MEASURES_TO_FLIP_BVS_MIRROR_OR_COUNTER && generators[curGen]->atProgressionEnd() )
            {
                measureBVSCount = 0;
                bvsMirror = chooseRandom() <= 0.5;
                percBvsMirror = chooseRandom() <= 0.5;
            }

            //if at the beginning or near end-ish, bvs should always mirror so that music stops when dancers stop
            if(where_in_song_structure==0 || beatTimer->getTimeInSeconds() >= 4.5*60)
            {
                bvsMirror = true;
                percBvsMirror = true;
            }
        }
        
        double chooseRandom()
        {
            double choose =((double) std::rand()) / ((double) RAND_MAX);
            return choose;
        }
        
        //make busy accompaniments less likely and harder to invoke
        int scaleBVSFrom20to5(int whichBVS, PerceptualEvent *e)
        {
            //ok this is hard-coded yikes but will fix later...
            std::vector<double> cutoffs = {4.0/20.0, 9.0/20.0, 14.0/20.0, 17.0/20.0};
            return e->getNonLinearScalingbyFiat(cutoffs, 5.0, (double) whichBVS);
        }
        
        void makeBothFivesLessLikely()
        {
            double chooseRemainFive = chooseRandom();
            if(chooseRemainFive > 0.25) // 75% likely that one will be modified to a 2.
            {
                if(fiveStopped) //five was also stopped previously so suppressed the previously suppreseed bvs value
                {
                    if(bvsChangedTo2) bvs = 2;
                    else percbvs = 2;
                }
                else
                {
                    double chooseBVS = chooseRandom();
                    if(chooseBVS > 0.5)
                    {
                        bvs = 2;
                    }
                    else percbvs = 2;
                }
                bvsChangedTo2 = bvs == 2;
                fiveStopped = true;
            }
            else fiveStopped = false;
        }
        
        void determineBVS(float seconds)
        {
            int lastPBVS = percbvs; //saves last value to check for change.
            int lastBVS = bvs;
            
            float accompwindow = 2;
            int origbvs = findBusySparse(accompwindow, seconds);
            PerceptualEvent *ev = findBusySparseSchema();

            //send dancer bvs to max
            ci::osc::Message msg;
            msg.setAddress(BUSY_SPARSE_DANCERS);
            msg.append(scaleBVSFrom20to5(origbvs, ev));
            harmonyMessages.push_back(msg);
            
//            std::cout << "Couple Busy Sparse: " <<  origbvs << " range:" << ev->getMinMood() << "-" << ev->getMaxMood() << std::endl;
            setBVSMirroring();
            if(!bvsMirror)
            {
                //then do opposite
                bvs = ev->getMaxMood() + origbvs * -1 + ev->getMinMood(); //max + bvs*-1 + min
            } else bvs = origbvs;
            if(!percBvsMirror)
            {
                percbvs = ev->getMaxMood() + origbvs * -1 + ev->getMinMood(); //max + bvs*-1 + min --
            }  else percbvs = origbvs;
            
            //favors  low and middle values over higher ones.
            bvs = scaleBVSFrom20to5(bvs, ev);
            percbvs = scaleBVSFrom20to5(percbvs, ev);

            //if both bvs are 5, change to something else 75% of the time -- include 4
            if(percbvs >= 4 && bvs >= 4)
                makeBothFivesLessLikely();
            
            makePercLessLikely(lastPBVS);
            
            percBvsChanged = percbvs != lastPBVS;
        
            //if bvs did go from 0 to 5 or 4, then add a fill
//            addFillsAndSmoothToZeros(lastBVS); //ok, try
        }
        
        void makePercLessLikely(int last)
        {
            if(beatTimer->getTimeInSeconds() < 60 ) //none at beginning
                percbvs = 1;
            
            if(percbvs > last ) //only escalate  60% of the time conditions are for it
            {
                double choose = chooseRandom();
                if(choose >  0.6)
                    percbvs =  last;
            }
        }
        
//        void addFillsAndSmoothToZeros(float lastBVS)
//        {
//            //if bvs did go to or from 1, then add a fill or close
//            //only go to 0 at progression end to decreas awkwardness
//
//
//                if( bvs > lastBVS && lastBVS == 1 )
//                {
//                    if(generators[curGen]->atProgressionEnd())
//                        generators[curGen]->resetChordIndex(); //so that when it comes in, it comes in on 1
//
//                    ci::osc::Message msg;
//                    msg.setAddress(EXPMUSIC_INTROFILL);
//                    harmonyMessages.push_back(msg);
//                    pauseForFillOrClose = true;
//                }
//                else if(bvs < lastBVS && bvs == 1)
//                {
//                    if(generators[curGen]->atProgressionEnd())
//                    {
//                        ci::osc::Message msg;
//                        msg.setAddress(EXPMUSIC_CLOSEFILL);
//                        harmonyMessages.push_back(msg);
//                        pauseForFillOrClose = true;
//                    }
//                } else bvs = 2;
//        }
        
        void createSampleHarmonyMessages() //now always send -- will filter on max side - since always need these messages for ritard -
        {
            //do not have for section 3 yet
            if (curSection == 3) return;
            
//            if(bvs >=3)
//            {
                ci::osc::Message msg;
                msg.setAddress(EXPMUSIC_HARMONY);
                msg.append(curGen);
                msg.append(generators[curGen]->getCurHarmony());
            
                //samplePlay -- 0 -- none, 1 - bandoneon, 2 - guitar, 3 - both
                if( bvs >= 5 )
                {
                    sampleplay = 3;
                }
                else if( bvs == 4 )
                {
                    if(sampleplay == 0)
                    {
                        double shouldplay = chooseRandom(); //both?
                        if(shouldplay < 0.33 ) sampleplay = 3;
                        else if(shouldplay < 0.66) sampleplay = 1;
                        else sampleplay =  2;
                    }
                }
                else if(bvs == 3)
                {
                    if(sampleplay == 0 )
                    {
                        double shouldplay = chooseRandom(); //both?
                        if(shouldplay < 0.33 ) sampleplay = 0;
                        else if(shouldplay < 0.66) sampleplay = 1;
                        else sampleplay =  2;
                    }
                    else if(sampleplay == 3)
                    {
                        double shouldplay = chooseRandom(); //both?
                        if(shouldplay < 0.5 ) sampleplay = 1;
                        else sampleplay =  2;
                    }
                }
                else
                {
                    sampleplay = 0;
                }
            
                msg.append(sampleplay); //should play accord samples?
                msg.append(curSection); //the current section
                harmonyMessages.push_back(msg);
//            }
        }
        
        void getChordGenerationNotes(float seconds)
        {
            if( generators[curGen]->atProgressionEnd() && curSection == 1 )
            {
                double choose = chooseRandom();
                if(choose < 0.5) curGen = 1; else curGen = 0;
            } else curGen = 0;
            
            determineBVS(seconds);
            
//            std::cout << "bvs :" << bvs << std::endl ;
            
//            bvs = 5;
            pauseForFillOrClose = false;
            
            if( !pauseForFillOrClose )
            {
                //ok... well for now just add lines for each bvs step
                if(bvs >= 2) notes.push_back(generators[curGen]->getNextChord());
                if(bvs >= 3) notes.push_back(generators[curGen]->getBass());
                if(bvs >= 4) notes.push_back(generators[curGen]->getTop());
                createSampleHarmonyMessages();
//                std::cout << "getting notes:" << notes.size();
            }
            else pauseForFillOrClose = false;
        
            //we'll see... for 5, maybe send commands to the percussion parts...
            
        }
        
        void setMelodySectionDecider(GeneratedMelodySection *mel)
        {
            sectionDecisionMaker = mel;
        }
        
        void changeSectionIfNeeded(float seconds)
        {
            if(sectionDecisionMaker != NULL) //getting section changes from somewhere else ------- TODO: FIX SINCE HAVE ANOTHER STRUCT FOR THIS in other tango system
            {
                curSection  = sectionDecisionMaker->getSection();
                where_in_song_structure = sectionDecisionMaker->getWhereInSong();
                generators.clear();
                generators = generatorsEaSection[curSection-1];
                
                if(where_in_song_structure < expInstrumentsforSections.size())
                {
                    //create a message to send re: main & top instrumentation
                    ci::osc::Message msg;
                    msg.setAddress(EXPMUSIC_ACCOMP_INSTRUMENT);
                    
                    //add instruments - first main, then bass, then top
                    for(int i=0; i<expInstrumentsforSections.size(); i++){
                        msg.append(expInstrumentsforSections[i].at(where_in_song_structure));
                    }
                    harmonyMessages.push_back(msg);

                }
//                std::cout << "Number of current generators: " << generators.size() << std::endl;
            }
        }
        
        void defaultExpInstrumentsforSections()
        {
            //these correspond to midi channels in Kontakt on the max side
            std::vector<int> mainChords = { 3, 5, 3, 16, 3, 3 };
            std::vector<int> bassIns = { 4, 4, 16, 16, 4, 4  };
            std::vector<int> topIns = { 5, 3, 3, 5, 3, 5  };

            expInstrumentsforSections.push_back(mainChords);
            expInstrumentsforSections.push_back(bassIns);
            expInstrumentsforSections.push_back(topIns);

        }
        
        void setExpInstrumentsforSections(std::vector<std::vector<int>> ins)
        {
            expInstrumentsforSections = ins;
        }
    
        //has to be updated with the harmony/section profile
        virtual void update(float seconds = 0 )
        {
            notes.clear();
            

            
            if( beatTimer->isOnBeat(0.0, seconds) ) //exactly on beat
            {
                beatsPlayed++;
            }

            if( beatsPlayed >= BEATSPERMEASURE) //TODO: fix hardcoding
            {
                beatsPlayed = 0;
                shouldStartFile = true;
                measureBVSCount++;
                
                changeSectionIfNeeded(seconds);
                getChordGenerationNotes(seconds);

                //switch the constant instrument
//                if( phraseStart )
//                {
//                    phraseStart = false;
//                    
//                    continuingInstrumentsThroughPhrase.clear();
//                    Orchestra *curOrch = curSoundFile->getOrchestration();
//                    //                    assert( !curOrch->empty() );
//                    for(int i=0; i<maxInstrumentsToHoldConstantThroughPhrase && i<curOrch->size(); i++)
//                    {
//                        continuingInstrumentsThroughPhrase.addInstrument(curOrch->getInstrViaIndex(i));
//                    }
//                }
                
            } else shouldStartFile = false;
        };
        
        std::vector<MidiNote> getBufferedNotes(int i=0)
        {
//            std::cout << "harmony notes..." << notes[i].size();
            return notes[i];
        }
        size_t voiceCount()
        {
            return notes.size();
        }
        
        virtual std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            //ok! now we are actually sending osc
            if(percBvsChanged)
            {
                ci::osc::Message msg;
                msg.setAddress(BUSY_SPARSE_PERCUSSION);
                msg.append(percbvs);
                msgs.push_back(msg);
            }
            
            if(!harmonyMessages.empty())
            {
                for(int i=0; i<harmonyMessages.size(); i++)
                    msgs.push_back(harmonyMessages[i]);
            }
            harmonyMessages.clear();

            
            return msgs; 
        }
    };
    
    class ExperimentalMusicPlayer : public MusicPlayer, public mm::MidiSequencePlayerResponder
    {
    protected:
        MidiOutUtility midiOut; //for now have the player own it... hmmmmmmmm....
        std::vector<mm::MidiSequencePlayer *> players;
        std::vector<int> deletePlayerTag;
        double ticksPerBeat;
        bool sendMidi;


    public:
        ExperimentalMusicPlayer() : MusicPlayer()
        {
            main_melody = NULL;
            deletePlayerTag.clear(); //because of the nature of the errors...
            sendMidi = true;
        }
        
        void addGeneratedMelodySection(GeneratedMelodySection *section)
        {
            if(main_melody==NULL)
            {
                setMainMelody(section);
            }
            else
            {
                addCounterMelody(section);
            }
        }
        
        void startMidi()
        {
            sendMidi = true;
        }
        
        void stopMidi()
        {
            sendMidi = false;
        }
        
//        virtual void update(float seconds = 0)
//        {
//            MusicPlayer::update(seconds);
//
//            //ok, send the midi now... may change tho...
//            
//            
//        }
        
        void resetSong()
        {
            if(main_melody == NULL)
                std::cout << "Cannot reset. No assigned melody\n";
            else
                ((GeneratedMelodySection *)main_melody)->resetSong();
        }
        
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
            

            for(int i=0; i<c_melodies.size(); i++)
            {
                c_melodies[i]->update(hsprofile, seconds);
                curMelodyOrchestration.addfromOtherOrchestra( c_melodies[i]->getOrchestration() );
            }
                
            for(int i=0; i<accompaniments.size(); i++)
            {
                if( main_melody->isStartOfPhrase() )
                    accompaniments[i]->startPhrase();
                ((GeneratedAccompanmentSection *)accompaniments[i])->update(seconds);
            }
//
//          //TODO: OK FIX SOON!!!!!
//          for(int i=0; i<ornaments.size(); i++)
//          {
//             ornaments[i]->update(hsprofile, &curMelodyOrchestration, seconds);
//          }
//        }
            curHarmonyProfile.reset();
            curHarmonyProfile = hsprofile;
        
            if(sendMidi) sendMidiMessages();
            deleteDeadPlayers();

    };
        
        virtual void playerStopped(int tag)
        {
//            std::cout << "putting in queue to delete...\n";
            
            //cannot delete immediately upon this event call as will cause seg fault shit since this would be deleleting the object that is calling that function.
            deletePlayerTag.push_back(tag);

        }
        
        //so, delete these after 10 or 12
        void deleteDeadPlayers()
        {
            while (deletePlayerTag.size() > MAX_DEAD_PLAYERS)
            {
                int tag = deletePlayerTag[0];
                int i=0;
                bool found = false;
                while (!found && i<players.size())
                {
                    found = players[i]->getTag() == tag;
                    i++;
                }

                if(found)
                {
                    delete players[i-1];
                    players.erase(players.begin()+i-1);
                }
                deletePlayerTag.erase(deletePlayerTag.begin());
            }
            
            
        }
    
    
        virtual void sendNoteSeq(std::vector<MidiNote> notes, int channel )
        {
            if(notes.size() <= 0) return;
            
            mm::MidiSequencePlayer *player = new mm::MidiSequencePlayer(*midiOut.getOut());
            
            //send the first note out immediately
            midiOut.send(notes[0], channel);
            
            for(int i=1; i<notes.size(); i++)
            {
//                if(notes[i].tick > 0){
                    //mm::MessageType::NOTE_ON
                    //        MidiMessage(const uint8_t b1, const uint8_t b2, const uint8_t b3, const double ts = 0) : timestamp(ts) { data = {b1, b2, b3}; }

                    std::shared_ptr<mm::MidiMessage> m(mm::MakeNoteOnPtr(channel,notes[i].pitch, notes[i].velocity));
//                    std::cout << "Sending to sequencer: note:" <<notes[i].pitch << "," << notes[i].velocity << std::endl;
                    std::shared_ptr<mm::MidiPlayerEvent> ev(new mm::MidiPlayerEvent(notes[i].tick, m, 1)); //MidiPlayerEvent(double t,std::shared_ptr<MidiMessage> m, int track)
                    
                    ev->channel = channel;
                    ev->tick = notes[i].tick;
                    
                    player->addTimestampedEvent(1, time(NULL), ev);
                    
                    
//                }
//                else midiOut.send(notes[i], channel);
            }
            
//            if(notes.size() <= 1) return ;
            
            player->setBeatTiming(main_melody->getTimer());
            player->setTag(players.size());
            player->setResponder(this);
            player->setTicksPerBeat(ticksPerBeat);
            player->start(); //start it
            players.push_back(player);
        }
        
        
        //OK this is bogus and ridic refactor ASAP
        virtual void sendMidiMessages()
        {
            //collect all of the current midi notes
            std::vector<MidiNote> notes;
            if(main_melody != NULL)
            {
                notes = ((GeneratedMelodySection *) main_melody)->getBufferedNotes();
            }
            ticksPerBeat = ((GeneratedMelodySection *) main_melody)->getTicksPerBeat();
            
            //now send them
            sendNoteSeq(notes, 1);
            
//            std::cout << "Sending Follower notes:" << notes.size();

            
            notes.clear();
            
            for(int i=0; i<c_melodies.size(); i++)
            {
                notes = ((GeneratedMelodySection *) c_melodies[i])->getBufferedNotes();
                
                sendNoteSeq(notes, 2);
//                std::cout << "Sending Leader notes:" << notes.size();
                
                notes.clear();
            }
            
            for(int i=0; i<accompaniments.size(); i++)
            {
                for(int j=0; j<((GeneratedAccompanmentSection *) accompaniments[i])->voiceCount(); j++)
                {
                    notes.clear();
                    notes = ((GeneratedAccompanmentSection *) accompaniments[i])->getBufferedNotes(j);
                    sendNoteSeq(notes, 3+j); //TODO: change for multiple accompaniment sections, altho not relevant
                }
            }
        }
        

        virtual std::vector<ci::osc::Message> getOSC()
        {
            //probably do nothing here... we'll see
            std::vector<ci::osc::Message> msgs = MusicPlayer::getOSC(); //this should return nothing.
            return msgs;
        }
        
 
    

    };
}

#endif /* ExperimentalMusicPlayer_h */
