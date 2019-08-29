//
//  MeasuredEntities.h
//  VideoAndOSCLab
//
//  Created by courtney on 10/22/18.
//

#ifndef MeasuredEntities_h
#define MeasuredEntities_h

namespace CRCPMotionAnalysis {

//who is being measured? or a collection or who or what's? --
//this sets up all the ugens and filters and organizes them according to what they represent in real life. For this example, I am only
//defining a hand. In order to use for yourselves, inherit from this class & add to BodyPart -- or rename it if you are tracking a non-human or non-animal.
    
//ok... will need to refactor the way I meant to do so in the beginning
//create a dancer of class entity .. change this class to body part tho... l
class BodyPartSensor : public UGEN
{
protected:
    std::vector<UGEN * > bodyPart; //assuming sensor is measuring some body part
     MocapDataVisualizer *bodyPartVisualizer, *notchBoneVisualizer; //assuming sensor is being held by the hand -- this will have to be a vector for mo' body parts, etc.
    
    FindPeaks *peaks;
    OutputSignalAnalysis *avgSignal;

    bool bodyPartInit = false; //whether we have set up the body part ugens
    int bodyPartID; //this will correspond to wii mote # or some OSC id'ing the sensor
    std::string whichBodyPart; //which bone it is
    MocapDeviceData::SendingDevice sendingDevice; //only accept data from one sending device (1st one)

    int count=0;
    
    //for the melodic/music output -- perhaps this is a temporary place...
    MelodyGenerator *melodyGenerator;
    std::vector<FactorOracle *> fo;
    
public:
    enum BodyPart{ HIP=0, CHESTBOTTOM=1, LEFTUPPERARM=2, LEFTHAND=3, LEFTFOREARM=4,
        RIGHTUPPERARM=5, RIGHTFOREARM=6, RIGHTHAND=7  }; //add body parts here. Doesn't have to be a body I suppose but you get what I mean.. --> need to change
    

    BodyPartSensor(){
        bodyPartInit = false;
    };
    
    bool isInit()
    {
        return bodyPartInit;
    };

    void setBoneID(int id_)
    {
        bodyPartID = id_;
    }
    
    void addSensor(int idz, SensorData *sensor, MocapDeviceData::SendingDevice device)
    {
        //which body part is the sensor of?
        whichBodyPart = sensor->getDeviceID();  //yip for now.
        bodyPartInit = true;
        bodyPartID = idz;
        
        //add all the basic signal analysis that happens for ea. sensor
        InputSignal *signal_input = new InputSignal(idz); //1. create an input
        signal_input->setInput(sensor);
        bodyPart.push_back( signal_input );
        
        //add an averaging filter
        //AveragingFilter(SignalAnalysis *s1, int w=10, int bufsize=48, int sensorID=0, std::string whichPart="", bool sendOSC=false )
        
        AveragingFilter *avgfilter = new AveragingFilter(signal_input, 3, 16, idz, whichBodyPart, true);
        bodyPart.push_back( avgfilter );
        avgSignal = avgfilter;
        
        Derivative *derivative = new Derivative(avgfilter, 16, idz, whichBodyPart, true);
        bodyPart.push_back( derivative );
  
        //add the visualizer
        bodyPartVisualizer = new MocapDataVisualizer( avgfilter );
        bodyPart.push_back( bodyPartVisualizer );
        
        //add the notch bone angle visualizer
        if(sensor->getDeviceType() == MocapDeviceData::MocapDevice::NOTCH)
        {
            notchBoneVisualizer = new MocapDataVisualizerNotchBonePosition( avgfilter );
            bodyPart.push_back( notchBoneVisualizer );
        }
        
        //ok, add a melody generator. why not? //9/13/2019
        loadGeneratedMelodies();
        
        //TODO: calibrate peak detection for notches - 6/14/2019
        
        //add peak detection... note this is a bit hacky since I am relying on the the fact that the hand visualizer has already scaled this accel.
        double THRESH_DEFAULT = 0.6; //peak must exceed this value to report a peak  -- currently calibrated for scope1 of syntienOSC from iphone
//        double THRESH_DEFAULT = 0.002; //peak must exceed this value to report a peak  -- currently calibrated for scope1 of syntienOSC from iphone

//        if(sensor->getDeviceType() == MocapDeviceData::MocapDevice::WIIMOTE)
//        {
//            THRESH_DEFAULT  =  0.002; //this will detect after a rather hard shake - raise or lower this number to make it easier to create a peak.
//        }
        
        const double WAIT_BETWEEN_PEAKS_DEFAULT = 0.3; //the time to wait between declaring another peak has happened in seconds
                                               //you may want to vary this btw x, y, z instead of having the values for everything depending on how it is placed.
        peaks = new FindPeaks(WAIT_BETWEEN_PEAKS_DEFAULT,avgfilter, idz, whichBodyPart);
        peaks->setThreshes(THRESH_DEFAULT, THRESH_DEFAULT, THRESH_DEFAULT);
        peaks->setMelodyGenerator(melodyGenerator); 
        bodyPart.push_back(peaks);
        

        
        sendingDevice=device;
    }
    
    void loadGeneratedMelodies()
    {
        //for the melodic/music output -- perhaps this is a temporary place...
        melodyGenerator = new MelodyGenerator( 1, 1, 1 );
        
        std::string folder = "/Users/courtney/Documents/cycling rhythms vocal experiment-uptodate/scores/track 1 midi files/";
        std::string descant = "leftHandMelodyOptions.mid";
//        std::string soprano = "firstShotFullMelody.mid";
//        std::string tenor = "firstShotTenor.mid";

        //TODO: put these in the same octave...
        //create new melody generator section -- TODO: REFACTOR!!!!!!!!
        FactorOracle *f = new FactorOracle();
        f->train(folder + descant, 1);
//        f->train(folder + soprano, 2);
//        f->train(folder + tenor, 2);
        
        melodyGenerator->addGeneratorAlgorithm(f);
        fo.push_back(f);
        
        melodyGenerator->turnOn1to1();
        bodyPart.push_back(melodyGenerator);
    };
    
    inline OutputSignalAnalysis *getAvgSignal()
    {
        return avgSignal;
    };
    
    inline std::string getWhichBodyPart()
    {
        return whichBodyPart;
    };
    
    inline MelodyGenerator *getMelodyGenerator()
    {
        return melodyGenerator;
    }
    
    //do all the drawing here -- but its container class will draw it, kz
    virtual void draw()
    {
//        bodyPartVisualizer->draw();
//        notchBoneVisualizer->draw();
    }
    
    //if we wanted to gather & send OSC from our ugens or send our own osc.. do it here.
    virtual std::vector<ci::osc::Message> getOSC()
    {
        std::vector<ci::osc::Message> msgs;
        std::vector<ci::osc::Message> nmsgs;
        
        //ok I'll do it
        for (int i=0; i<bodyPart.size(); i++)
        {
            nmsgs = bodyPart[i]->getOSC();

            for(int j=0; j<nmsgs.size(); j++)
            {
                msgs.push_back(nmsgs[j]);
            }
        }
        return msgs;
    };
    
    //this is mostly just for calibration since during normal use we would just send OSC somewhere -- well it depends.
    bool peak()
    {
        //is there a peak at this body part?
//        this tests the peak -- the count is to differentiate it from the last peak
        bool aPeak = peaks->getCombinedPeak();
        if( aPeak )
        {
            std::cout << whichBodyPart << " peak:" << count << std::endl ;
            count++;
        }
        return aPeak;
    }
    
    void setPeakThresh(float x, float y, float z)
    {
        peaks->setThreshes(x, y, z);
    }
    
    //to decrease just use negative values or to only change 1 zero out the others. easy peasy dudes.
    void increasePeakThresh(float xAmt, float yAmt, float zAmt)
    {
        peaks->increaseXThresh(xAmt);
        peaks->increaseYThresh(yAmt);
        peaks->increaseZThresh(zAmt);
        
        peaks->printThreshes();
    }
    
    //update all the ugens we own. all of them that need updating..
    virtual void update(float seconds = 0)
    {
        for(int i=0; i<bodyPart.size(); i++){
            bodyPart[i]->update(seconds);
        }
        
    };

};
    
    class Entity : public UGEN
    {
    protected:
        std::vector<BodyPartSensor * > bodyParts; //assuming sensor is measuring some body part
        NotchBoneFigure *figure;
        std::vector<FigureMeasure * > figureMeasures;

    public:
        Entity() : UGEN()
        {
//            double w = ci::app::getWindowWidth() * 0.25;
//            int i=0; //(>_<)
//            
//            for (Axis axis : AxisList)
//            {
//                figure.push_back(new NotchBoneFigureVisualizer(axis, w + w*i));
//                i++; //(>_<)
//            }
            
//            double w = ci::app::getWindowWidth() * 0.5;
            figure = new NotchBoneFigure();
            
            figureMeasures.push_back(new ContractionIndex(figure));
            figureMeasures.push_back(new ArmHeight(figure));

        }
        bool bodyPartExists(std::string whichPart)
        {
            return bodyPartIndex(whichPart)!=-1;
        }
        
        int bodyPartIndex(std::string whichPart)
        {
            int i=0;
            bool found = false;
            while (i < bodyParts.size() && !found)
            {
                found = !whichPart.compare(bodyParts[i]->getWhichBodyPart());
                i++;
            }
            if(found)
                return i-1;
            else return -1;
        }
        
        void addBodyPart(BodyPartSensor *part)
        {
//            for(NotchBoneFigureVisualizer *f: figure)
//                f->setInputSignal(part->getWhichBodyPart(), part->getAvgSignal());
            
            figure->setInputSignal(part->getWhichBodyPart(), part->getAvgSignal());
            part->setBoneID(figure->getBoneID(part->getWhichBodyPart()));
            bodyParts.push_back(part);
        }
        
        //this is maybe toooo much
        BodyPartSensor *getPartWithID(int id_)
        {
            return bodyParts[bodyPartIndex(getBoneName(id_))];
        }
        
        std::string getBoneName(int id_)
        {
            return figure->getBoneName(id_);
        }
        
        //update all the ugens we own. all of them that need updating..
        virtual void update(float seconds = 0)
        {
            for(int i=0; i<bodyParts.size(); i++){
                bodyParts[i]->update(seconds);
                
                //only test hands for now
//                if( !bodyParts[i]->getWhichBodyPart().compare("LeftHand") ||
//                    !bodyParts[i]->getWhichBodyPart().compare("RightHand"))
//                {
//                    bodyParts[i]->peak(); //just print the peaks for noaw
//                }
                
            }
            
//            for(NotchBoneFigureVisualizer *f: figure)
//                f->update(seconds);
            figure->update(seconds);
            
            for(int i=0; i<figureMeasures.size(); i++)
                figureMeasures[i]->update(seconds);
        };
        
        void adjustPeakThreshes(std::string boneName, float xAmt, float yAmt, float zAmt )
        {
            int index = bodyPartIndex(boneName);
            if(index==-1)
            {
                std::cout << "AdjustPeakThreshes function error: " << boneName << "not found!!!!!!\n";
                return;
            }
            BodyPartSensor *part = bodyParts[index];
            part->increasePeakThresh(  xAmt,  yAmt,  zAmt );
        }
        
        void increasePeakThresh(std::string boneName, float amt=0.0001)
        {
            adjustPeakThreshes(boneName, amt, amt, amt);
        }
        
        void decreasePeakThresh(std::string boneName, float amt=0.0001)
        {
            adjustPeakThreshes(boneName, -amt, -amt, -amt);
        }
        
        //collect all the OSC messages and send them to calling class
        virtual std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            std::vector<ci::osc::Message> nmsgs;
            std::vector<ci::osc::Message> nmsgs2;
            
            for (int i=0; i<bodyParts.size(); i++)
            {
                nmsgs = bodyParts[i]->getOSC();
                
//                //only test hands for now
//                if( !bodyParts[i]->getWhichBodyPart().compare("LeftHand") ||
//                   !bodyParts[i]->getWhichBodyPart().compare("RightHand"))
//                {
//                    //get melody notes... for the hand only... for now
//
//                }
                
                for(int j=0; j<nmsgs.size(); j++)
                {
                    msgs.push_back(nmsgs[j]);
                }
            }
            
            for (int i=0; i<figureMeasures.size(); i++)
            {
                nmsgs2 = figureMeasures[i]->getOSC();
                
                for(int j=0; j<nmsgs2.size(); j++)
                {
                    msgs.push_back(nmsgs2[j]);
                }
            }
            
            return msgs;
        };
        
        //do all the drawing here.
        virtual void draw()
        {
            for (int i=0; i<bodyParts.size(); i++)
            {
                bodyParts[i]->draw();
            }
//            for(NotchBoneFigureVisualizer *f: figure)
//                f->draw();
            figure->draw();
            
            for (int i=0; i<figureMeasures.size(); i++)
            {
                figureMeasures[i]->draw();
            }

        }
    
    };
    
};

#endif /* MeasuredEntities_h */
