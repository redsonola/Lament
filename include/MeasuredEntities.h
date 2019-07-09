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
    
    
    int count=0;
public:
    enum BodyPart{ HIP=0, CHESTBOTTOM=1, LEFTUPPERARM=2, LEFTFOREARM=3, RIGHTUPPERARM=4, RIGHTFOREARM=5 }; //add body parts here. Doesn't have to be a body I suppose but you get what I mean.. --> need to change

    BodyPartSensor(){
        bodyPartInit = false;
    };
    
    bool isInit()
    {
        return bodyPartInit;
    };

    void addSensor(int idz, SensorData *sensor)
    {
        //which body part is the sensor of?
        whichBodyPart = sensor->getDeviceID();  //yip for now.
        
        std::vector<UGEN * > *bUgens; //the ugen vector to add ugens to
        bUgens = &bodyPart;
        bodyPartInit = true;
        bodyPartID = idz;
        
        //add all the basic signal analysis that happens for ea. sensor
        InputSignal *signal_input = new InputSignal(idz); //1. create an input
        signal_input->setInput(sensor);
        bUgens->push_back( signal_input );
        
        //add an averaging filter
        //AveragingFilter(SignalAnalysis *s1, int w=10, int bufsize=48, int sensorID=0, std::string whichPart="", bool sendOSC=false )
        
        AveragingFilter *avgfilter = new AveragingFilter(signal_input, 3, 16, idz, whichBodyPart, true);
        bUgens->push_back( avgfilter );
        avgSignal = avgfilter;
        
        Derivative *derivative = new Derivative(avgfilter, 16, idz, whichBodyPart, true);
        bUgens->push_back( derivative );
  
        //add the visualizer
        bodyPartVisualizer = new MocapDataVisualizer( avgfilter );
        bUgens->push_back( bodyPartVisualizer );
        
        //add the notch bone angle visualizer
        if(sensor->getDeviceType() == MocapDeviceData::MocapDevice::NOTCH)
        {
            notchBoneVisualizer = new MocapDataVisualizerNotchBonePosition( avgfilter );
            bUgens->push_back( notchBoneVisualizer );
        }
        
        //TODO: calibrate peak detection for notches - 6/14/2019
        
        //add peak detection... note this is a bit hacky since I am relying on the the fact that the hand visualizer has already scaled this accel.
        double THRESH_DEFAULT = 0.07; //peak must exceed this value to report a peak  -- currently calibrated for scope1 of syntienOSC from iphone
        if(sensor->getDeviceType() == MocapDeviceData::MocapDevice::WIIMOTE)
        {
            THRESH_DEFAULT  =  0.002; //this will detect after a rather hard shake - raise or lower this number to make it easier to create a peak.
        }
        
        const double WAIT_BETWEEN_PEAKS_DEFAULT = 0.35; //the time to wait between declaring another peak has happened in seconds
                                               //you may want to vary this btw x, y, z instead of having the values for everything depending on how it is placed.
        peaks = new FindPeaks(WAIT_BETWEEN_PEAKS_DEFAULT,avgfilter, idz);
        bUgens->push_back(peaks);
        peaks->setThreshes(THRESH_DEFAULT, THRESH_DEFAULT, THRESH_DEFAULT);
    }
    
    inline OutputSignalAnalysis *getAvgSignal()
    {
        return avgSignal;
    };
    
    inline std::string getWhichBodyPart()
    {
        return whichBodyPart;
    };
    
    
    //do all the drawing here.
    virtual void draw()
    {
//        bodyPartVisualizer->draw();
//        notchBoneVisualizer->draw();
    }
    
    //if we wanted to gather & send OSC from our ugens or send our own osc.. do it here.
    virtual std::vector<ci::osc::Message> getOSC()
    {
        std::vector<ci::osc::Message> msgs;
        
        //this tests the peak -- the count is to differentiate it from the last peak
        
//        if( peaks->getCombinedPeak() )
//        {
//            std::cout << "PEAK:" << count << std::endl ;
//            count++;
//        }
        
        //ok I'll do it
        for (int i=0; i<bodyPart.size(); i++)
        {
            std::vector<ci::osc::Message> nmsgs = bodyPart[i]->getOSC();

            for(int j=0; j<nmsgs.size(); j++)
            {
                msgs.push_back(nmsgs[j]);
            }
        }
        return msgs;
    };
    
    
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
        NotchBoneFigureVisualizer *figure;

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
            figure = new NotchBoneFigureVisualizer();

        }
        void addBodyPart(BodyPartSensor *part)
        {
//            for(NotchBoneFigureVisualizer *f: figure)
//                f->setInputSignal(part->getWhichBodyPart(), part->getAvgSignal());
            
            figure->setInputSignal(part->getWhichBodyPart(), part->getAvgSignal());
            bodyParts.push_back(part);
        }
        
        //update all the ugens we own. all of them that need updating..
        virtual void update(float seconds = 0)
        {
            for(int i=0; i<bodyParts.size(); i++)
                bodyParts[i]->update(seconds);
            
//            for(NotchBoneFigureVisualizer *f: figure)
//                f->update(seconds);
            figure->update(seconds);
            
        };
        
        //collect all the OSC messages and send them to calling class
        virtual std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            
            for (int i=0; i<bodyParts.size(); i++)
            {
                std::vector<ci::osc::Message> nmsgs = bodyParts[i]->getOSC();
                
                for(int j=0; j<nmsgs.size(); j++)
                {
                    msgs.push_back(nmsgs[j]);
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
        }
    
    };
    
};

#endif /* MeasuredEntities_h */
