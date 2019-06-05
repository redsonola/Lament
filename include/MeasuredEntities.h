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
class Entity : public UGEN
{
protected:
    std::vector<UGEN * > hand; //assuming sensor is being held by the hand
     MocapDataVisualizer *handVisualizer, *notchBoneVisualizer; //assuming sensor is being held by the hand -- this will have to be a vector for mo' body parts, etc.
    
    FindPeaks *peaks;

    bool handInit = false; //whether we have set up the hand ugens
    int handID; //this will correspond to wii mote # or some OSC id'ing the sensor
    std::string whichBodyPart; //which bone it is
    
    
    int count=0;
public:
    enum BodyPart{ HAND=0 }; //add body parts here. Doesn't have to be a body I suppose but you get what I mean.. --> need to change

    Entity(){
        handInit = false;
    };
    
    bool isInit()
    {
        return handInit;
    };

    void addSensorBodyPart(int idz, SensorData *sensor, BodyPart whichBody )
    {
        //which body part is the sensor of?
        whichBodyPart = sensor->getDeviceID();  //yip for now.
        
        std::vector<UGEN * > *bUgens; //the ugen vector to add ugens to
        switch (whichBody)
        {
            case HAND: //well it has to be currently, but ya know, in case this gets expanded...
                bUgens = &hand;
                handInit = true;
                handID = idz;
                break;
            default:
                break;
        };
        
        //add all the basic signal analysis that happens for ea. sensor
        InputSignal *signal_input = new InputSignal(idz); //1. create an input
        signal_input->setInput(sensor);
        bUgens->push_back( signal_input );
        
        //add an averaging filter
        //AveragingFilter(SignalAnalysis *s1, int w=10, int bufsize=48, int sensorID=0, std::string whichPart="", bool sendOSC=false )
        
        AveragingFilter *avgfilter = new AveragingFilter(signal_input, 3, 16, idz, whichBodyPart, true);
        bUgens->push_back( avgfilter );
        
        Derivative *derivative = new Derivative(avgfilter, 16, idz, whichBodyPart, true);
        bUgens->push_back( derivative );
  
//        //add the visualizer
        handVisualizer = new MocapDataVisualizer( avgfilter );
        bUgens->push_back( handVisualizer );
        
        //add the notch bone angle visualizer
        if(sensor->getDeviceType() == MocapDeviceData::MocapDevice::NOTCH)
        {
            notchBoneVisualizer = new MocapDataVisualizerNotchBonePosition( avgfilter );
            bUgens->push_back( notchBoneVisualizer );
        }
        
        //add peak detection... note this is a bit hacky since I am relying on the the fact that the hand visualizer has already scaled this accel.
        //fix if horrified.
        //note that it takes the avgfilter as input -- for some applications taking 1 or more derivatives of the average and then finding the peaks could be useful
        //also note wiimote is giving you rotational position whereas the phone is giving you actual acceleration -- which affects peaks, etc.
        //if you want to send OSC -- create those messages in the findpeaks ugen getOSC().
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
    
    //do all the drawing here.
    virtual void draw()
    {
        handVisualizer->draw();
        notchBoneVisualizer->draw();
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
        for (int i=0; i<hand.size(); i++)
        {
            std::vector<ci::osc::Message> nmsgs = hand[i]->getOSC();

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
        for(int i=0; i<hand.size(); i++)
            hand[i]->update(seconds);
        

        
    };

};
    
};

#endif /* MeasuredEntities_h */
