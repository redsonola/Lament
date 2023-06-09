/***
 Courtney Brown
 
 Jan. 2019
 
 This is the start of my new work -- fever rhythm cycle
 
 ***/

//osc messages
#define ELAPSED_FRAMES_ADDR "/VideoAndOSCLab/elapsedFrames"
#define ELAPSED_SECS_ADDR "/VideoAndOSCLab/elapsedSeconds"
#define SYNTIEN_MESSAGE "/syntien/motion/1/scope1"
#define WIIMOTE_ACCEL_MESSAGE_PART1 "/wii/"
#define WIIMOTE_ACCEL_MESSAGE_PART2 "/accel/pry"
#define WIIMOTE_BUTTON_1 "/wii/1/button/1"
#define NOTCH_MESSAGE "/Notch/BonePosAndAccel"
#define NOTCH_PEAK "/Notch/Peak"
#define DERIVATIVE_OSCMESSAGE "/CBIS/Derivative"
#define SIGAVG_OSCMESSAGE "/CBIS/Average"
#define CI_OSCMESSAGE "/CBIS/CI" //send contraction index
#define ARMHEIGHT_OSCMESSAGE "/CBIS/ArmHeight" //send relative arm height index
#define VERTICALITY_OSCMESSAGE "/CBIS/Verticality" //send verticality
#define MIDINOTE_OSCMESSAGE "/CBIS/MidiNote"


#define SEND_TO_WEKINATOR 1
#define WEK_MESSAGE "/wek/inputs"

//for sending busy spase to max 7
#define BUSY_SPARSE_PERCUSSION "/InteractiveTango/BusySparse/Beats"
#define BUSY_SPARSE_DANCERS "/InteractiveTango/BusySparse/Dancers"
#define EXPMUSIC_HARMONY "/InteractiveTango/Experimental/Harmony"
#define EXPMUSIC_INTROFILL "/InteractiveTango/Experimental/HarmonyIntroFill"
#define EXPMUSIC_CLOSEFILL "/InteractiveTango/Experimental/HarmonyCloseFill"
#define EXPMUSIC_MELODY_INSTRUMENT "/InteractiveTango/Experimental/Melody/Instrument"
#define EXPMUSIC_ACCOMP_INSTRUMENT "/InteractiveTango/Experimental/Accompaniment/Instrument"
#define EXPMUSIC_SECTION "/InteractiveTango/Experimental/Section"


#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Capture.h" //needed for capture
#include "cinder/Log.h" //needed to log errors

#include <sstream>
#include <math.h>
#include <algorithm>

#include "Osc.h"

#include "Containers.h"
#include "BeatTiming.h"
#include "MotionCaptureData.h"
#include "Sensor.h"
#include "MotionAnalysisOuput.h"
#include "UGENs.h"


#include "PerceptualMappingScheme.h"
#include "PerceptualSchemas.h"

#include "Instruments.h"
#include "MidiUtility.h"
//#include "MusicPlayer.h"
//#include "ChordGeneration.h"
#include "MelodyGeneratorAlgorithm.h"
#include "MelodyGenerator.h"
#include "PeakDetection.h"

//#include "ExperimentalMusicPlayer.h"

#include "FactorOracle.h"


#include "MeasuredEntities.h"
#include "SaveOSC.h"

#define LOCALPORT 8886
#define LOCALPORT2 8887
#define DESTHOST "127.0.0.1"
#define DESTPORT 8888
#define WEKPORT 6448
#define LOCALPORT3 8889
#define LOCALPORT4 8890


#define REMOTELAPTOP_PORT 3333
#define REMOTELAPTOP_ADDRESS "192.168.1.88"

#define MAX_NUM_OF_WIIMOTES 6 //limitation of bluetooth class 2
#define PHONE_ID "7" //this assumes only one phone using Syntien or some such -- can modify if you have more...

using namespace ci;
using namespace ci::app;
using namespace std;

using Receiver = osc::ReceiverUdp;
using protocol = asio::ip::udp;

//This class demonstrates a 'hello, world' for the signal processing tree paradigm for motion capture
//Receives wiimote data, puts an averaging filter on it, then draws the data
//Also, w/o signal processing paradigm, this program has the functions to draw a phone but not implemented currently
class FeverRhythmCycleMain : public App {
public:
    FeverRhythmCycleMain();
    
    void setup() override;
    //    void mouseDown( MouseEvent event ) override;
    void keyDown( KeyEvent event ) override;
    void keyUp( KeyEvent event ) override;

    void mouseDrag( MouseEvent event ) override;
    void mouseDown( MouseEvent event ) override;

    
    void update() override;
    void draw() override;
    
protected:
    CaptureRef                 mCapture;
    gl::TextureRef             mTexture;
    SurfaceRef                 mSurface;
    
    osc::SenderUdp             mSender;
    osc::SenderUdp             mWekSender;
    osc::SenderUdp             mRemoteLaptopSender;
    
    //moving to 3d drawing
    void drawGrid(float size, float step);
    ci::CameraPersp     mCamera;
    ci::vec3 mLookAt, mEyePoint, mDefaultEyePoint;
    void updateCamera();
    void initCamera();
    MouseEvent mLastMouseEvent;
    bool mShiftKeyDown;
    
    bool mChangePeakThresholdMode;
    CRCPMotionAnalysis::BodyPartSensor::BodyPart mWhichThresh;
    
    void printKeyboardMenu();

    void sendOSC(std::string addr, float value);
    
    Receiver mReceiver; //new
    std::map<uint64_t, protocol::endpoint> mConnections; //new
    
    //below for printing out iphone info
    std::vector<ci::vec2> points;
    std::vector<float>  alpha;
    
    void updatePhoneValues(const osc::Message &message);
    void updateWiiValues(const osc::Message &message);
    void updateNotchValues(const osc::Message &message);
    void createNotchMotionData(std::string _id, std::string who, std::string sendingDevice, std::vector<float> vals);
    void printSensors();

    void printNotchValues(const osc::Message &message);
    void addPhoneAndWiiData(const osc::Message &message, std::string _id);
    
    CRCPMotionAnalysis::SensorData *getSensor( std::string _id, int which, CRCPMotionAnalysis::MocapDeviceData::MocapDevice device, CRCPMotionAnalysis::MocapDeviceData::SendingDevice sdevice); // find sensor or wiimote in list via id
    std::vector<CRCPMotionAnalysis::SensorData *> mSensors; //all the sensors which have sent us OSC -- well only wiimotes so far
    std::vector<CRCPMotionAnalysis::BodyPartSensor *> mBodyParts;  //who are we measuring? change name when specifics are known.
    std::vector<CRCPMotionAnalysis::Entity *> mPeople;
    
    float seconds; //where we are
    
    //file
    CRCPMotionAnalysis::SaveOSC *saveOSC;
    CRCPMotionAnalysis::PlayOSC *playOSC;
    
    int testOSCNumber;
};

FeverRhythmCycleMain::FeverRhythmCycleMain() : mSender(LOCALPORT, DESTHOST, DESTPORT), mReceiver( LOCALPORT2 ), mWekSender(LOCALPORT3, DESTHOST, WEKPORT), mRemoteLaptopSender(LOCALPORT4, REMOTELAPTOP_ADDRESS, REMOTELAPTOP_PORT)
{
    
}

//outdated vestige
void FeverRhythmCycleMain::sendOSC(std::string addr, float value)
{
    osc::Message msg;
    msg.setAddress(addr);
    msg.append(value); //adds a parameter
    mSender.send(msg);
}

//this has not been implemented into signal tree paradigm yet. ah well. TODO: implement as such
void FeverRhythmCycleMain::updatePhoneValues(const osc::Message &message)
{
//    std::cout << "received\n";
    addPhoneAndWiiData(message, PHONE_ID);
}

//gets data from osc message then adds wiimote data to sensors
void FeverRhythmCycleMain::addPhoneAndWiiData(const osc::Message &message, std::string _id)
{
    CRCPMotionAnalysis::MocapDeviceData *sensorData;
    std::string dID = _id ;
    int which = std::atoi(_id.c_str()); //convert to int
    
    
    if(!dID.compare(PHONE_ID)){
        sensorData = new CRCPMotionAnalysis::IPhoneDeviceData();
    }
    else{
        sensorData = new CRCPMotionAnalysis::MocapDeviceData();
    }

    
    CRCPMotionAnalysis::SensorData *sensor = getSensor( _id, which, sensorData->getDeviceType(), CRCPMotionAnalysis::MocapDeviceData::SendingDevice::UNSPECIFIED );
    
    //set time stamp
    sensorData->setData( CRCPMotionAnalysis::MocapDeviceData::DataIndices::TIME_STAMP, seconds ); //set timestamp from program -- synch with call to update()
    
    //add accel data
    for(int i= 0; i<3; i++)
        sensorData->setData(CRCPMotionAnalysis::MocapDeviceData::DataIndices::ACCELX+i, message.getArgFloat(i));
    
    sensor->addSensorData(sensorData);
}

//return sensor with id & or create one w/detected id then return that one
CRCPMotionAnalysis::SensorData *FeverRhythmCycleMain::getSensor( std::string _id, int which, CRCPMotionAnalysis::MocapDeviceData::MocapDevice device, CRCPMotionAnalysis::MocapDeviceData::SendingDevice sDevice )
{
    
    bool found = false;
    int index = 0;
    
//    printSensors();
    
    while( !found && index < mSensors.size() )
    {
        found = mSensors[index]->same( _id, which, sDevice );
        index++;
    }
    
    if(found)
    {
//        if(!_id.compare("ChestBottom")){
//            std::cout << mSensors[index-1]->getDeviceID() << "==" << _id << "    " ;
//            std::cout << mSensors[index-1]->getSendingDeviceString() << "==" ;
//            if (sDevice == CRCPMotionAnalysis::MocapDeviceData::SendingDevice::ANDROID)
//                std::cout << "Android\n";
//            else std::cout << "iOS\n";
//        }
        return mSensors[index-1];
    }
    else
    {
        CRCPMotionAnalysis::SensorData *sensor = new CRCPMotionAnalysis::SensorData( _id, which, device, sDevice );
        
        mSensors.push_back(sensor);
        
        //TODO: ok, handle multiple entities later -- (use the "who" id in the OSC message)
        if(mPeople.size() <= 0)
        {
            mPeople.push_back(new CRCPMotionAnalysis::Entity());
        }
        
        //okay now test if this body part already exists in the person
        //this is bc do not want to use different sensors from the same bodypart (eg. chest) which
        // bc it is a root bone will need to be duplicated across phones
        //ergo this info will be discarded...
        if(mPeople[0]->bodyPartExists(sensor->getDeviceID()))  //yip for now.
        {
            std::cout << "Note that " << sensor->getDeviceID() << " exists already. Not using data from device ";
            if (sDevice == CRCPMotionAnalysis::MocapDeviceData::SendingDevice::ANDROID)
                std::cout << "Android\n";
            else std::cout << "iOS\n";
            
            return sensor;
        }
        
            
        //add to 'body part' the data structure which can combine sensors. It currently only has one body part so it is simple.
        
        //TODO: check the UGEN / averaging sensor ID
        int bodyPartID  = mSensors.size()-1;
        CRCPMotionAnalysis::BodyPartSensor *bodyPart = new CRCPMotionAnalysis::BodyPartSensor();
        bodyPart->addSensor(bodyPartID, sensor, sDevice);  //note that this should change if using bones, etc.
        mBodyParts.push_back(bodyPart);
        
        mPeople[0]->addBodyPart(bodyPart);
        
        return sensor;
    }
}

void FeverRhythmCycleMain::printSensors()
{
    std::cout << "-------- SensorList: \n";

    for (CRCPMotionAnalysis::SensorData *sensor : mSensors)
    {
        if(sensor != NULL)
        {
            std::cout << "Bone: " << sensor->getDeviceID() ;
            std::cout << " Sending Device: " << sensor->getSendingDeviceString() << std::endl ;
        }
    }
    
}

//finds the id of the wiimote then adds the wiidata to ugens
void FeverRhythmCycleMain::updateWiiValues(const osc::Message &message)
{
    //get which wii
    std::string addr = message.getAddress();
    std::string pt1 = WIIMOTE_ACCEL_MESSAGE_PART1;
    int index = addr.find_first_of(pt1);
    std::string whichWii = addr.substr(index+pt1.length(), 1);
    addPhoneAndWiiData(message, whichWii);
}

void FeverRhythmCycleMain::printNotchValues(const osc::Message &message)
{
    std::string addr = message.getAddress();
    std::cout << addr;
    
    //so bad, basically cut and pasted from SaveOSC.h  --> needs refactoring
    for( int i=0; i<message.getNumArgs(); i++)
    {
        ci::osc::ArgType type_ = message.getArgType(i);
        std::cout << "," ;
        switch (type_) {
            case ci::osc::ArgType::INTEGER_32:
                std::cout << message.getArgInt32(i);
                break;
            case ci::osc::ArgType::FLOAT:
                std::cout << message.getArgFloat(i);
                break;
            case ci::osc::ArgType::DOUBLE:
                std::cout << message.getArgDouble(i);
                break;
            case ci::osc::ArgType::STRING:
                std::cout << message.getArgString(i);
                break;
            default:
                break;
        }
    }
//    std::cout << std::endl;

}

//handles notch data
void FeverRhythmCycleMain::updateNotchValues(const osc::Message &message)
{
    //print it out. Note that for each bone, you will want to create a new sensor
//    printNotchValues(message);
    
    /*** format of the notch OSC message
     1. Sample Rate (currently it is 20Hz - can be  changed as this is low)
     2. frame - which frame it is in cur. data grab
     
     Then for each bone:
        Bone name (eg, Root)
        acceleration - x, y, z
        then the different angles for each bone, for example the chest: 1. anterior/posterior tilt,  2. rotation left or right, and 3. lateral tilt -  left & right

     Only measured bones have all 6 values. Static bones w.o notches attached are reported but only have position info
     
     THis function also forwards data to wekinator - TODO: perhaps refactor out
     
     ***/
    
    std::string bone = "";
    std::string sendingDevice = "";
    std::string sender="";
    osc::Message wekMsg;
    
//    std::cout << message << std::endl;

/*
address: /Notch/BonePosAndAccel
    Sender Ip Address: 192.168.1.3
    <FLOAT>: 20
    <INTEGER_32>: 0
    <STRING>: Root
    <FLOAT>: 0
    <FLOAT>: 0
    <FLOAT>: 0
    <STRING>: Hip
    <FLOAT>: 0.00390107
    <FLOAT>: -0.0765789
    <FLOAT>: -0.00750544
    <FLOAT>: -0.00207619
    <FLOAT>: 0.0386578
    <FLOAT>: 0.0100631
    <STRING>: Tummy
    <FLOAT>: -0.0111514
    <FLOAT>: 0.230822
    <FLOAT>: 0.0350948
    <STRING>: ChestBottom
    <FLOAT>: -0.0280236
    <FLOAT>: -0.0667842
    <FLOAT>: 0.0630484
    <FLOAT>: -0.0170402
    <FLOAT>: 0.396716
    <FLOAT>: 0.0359006
    <STRING>: ChestTop
    <FLOAT>: -0.0191746
    <FLOAT>: 0.4962
    <FLOAT>: 0.025989
    <STRING>: LeftCollar
    <FLOAT>: 0.136829
    <FLOAT>: 0.480698
    <FLOAT>: 0.0385829
    <STRING>: LeftUpperArm
    <FLOAT>: -0.283832
    <FLOAT>: -0.0355547
    <FLOAT>: -0.135343
    <FLOAT>: 0.165057
    <FLOAT>: 0.226576
    <FLOAT>: 0.0566382
    <STRING>: LeftForeArm
    <FLOAT>: 0.278568
    <FLOAT>: 0.1125
    <FLOAT>: 0.0132517
    <FLOAT>: 0.164294
    <FLOAT>: -0.0829402
    <FLOAT>: 0.0876608
    <STRING>: LeftHand
    <FLOAT>: 0.160795
    <FLOAT>: -0.232138
    <FLOAT>: 0.102758
    <STRING>: RightCollar
    <FLOAT>: -0.174324
    <FLOAT>: 0.471908
    <FLOAT>: 0.0173598
    <STRING>: RightUpperArm
    <FLOAT>: 0.214931
    <FLOAT>: -0.00784064
    <FLOAT>: -0.329319
    <FLOAT>: -0.225098
    <FLOAT>: 0.220704
    <FLOAT>: 0.0217642
    <STRING>: RightForeArm
    <FLOAT>: -0.901787
    <FLOAT>: -0.342038
    <FLOAT>: 0.0815372
    <FLOAT>: -0.300555
    <FLOAT>: -0.0756913
    <FLOAT>: 0.0785047
    <STRING>: RightHand
    <FLOAT>: -0.333907
    <FLOAT>: -0.219269
    <FLOAT>: 0.106307
    <STRING>: Neck
    <FLOAT>: -0.0218852
    <FLOAT>: 0.622546
    <FLOAT>: 0.0134013
    <STRING>: Head
    <FLOAT>: -0.0256952
    <FLOAT>: 0.750411
    <FLOAT>: 0.0163005
    <STRING>: LeftHip
    <FLOAT>: 0
    <FLOAT>: 0
    <FLOAT>: 0
    <STRING>: LeftThigh
    <FLOAT>: 0
    <FLOAT>: -0.422
    <FLOAT>: 0
    <STRING>: LeftLowerLeg
    <FLOAT>: 0
    <FLOAT>: -0.865
    <FLOAT>: 0
    <STRING>: LeftFootTop
    <FLOAT>: 0
    <FLOAT>: -0.905
    <FLOAT>: 0.1047
    <STRING>: LeftFootFront
    <FLOAT>: 0
    <FLOAT>: -0.9395
    <FLOAT>: 0.1978
    <STRING>: LeftHeel
    <FLOAT>: 0
    <FLOAT>: -0.9285
    <FLOAT>: -0.0592
    <STRING>: RightHip
    <FLOAT>: 0
    <FLOAT>: 0
    <FLOAT>: 0
    <STRING>: RightThigh
    <FLOAT>: 0
    <FLOAT>: -0.422
    <FLOAT>: 0
    <STRING>: RightLowerLeg
    <FLOAT>: 0
    <FLOAT>: -0.865
    <FLOAT>: 0
    <STRING>: RightFootTop
    <FLOAT>: 0
    <FLOAT>: -0.905
    <FLOAT>: 0.1047
    <STRING>: RightFootFront
    <FLOAT>: 0
    <FLOAT>: -0.9395
    <FLOAT>: 0.1978
    <STRING>: RightHeel
    <FLOAT>: 0
    <FLOAT>: -0.9285
    <FLOAT>: -0.0592
*/
    
    if(SEND_TO_WEKINATOR)
    {
        wekMsg.setAddress(WEK_MESSAGE);
    }
    
//    std::cout << message << std::endl;
    
    sendingDevice = message.getArgString(1);
    sender = message.getArgString(0);

    for ( int i=3; i<message.getNumArgs(); i++)
    {
        ci::osc::ArgType type_ = message.getArgType(i);
        
        if(type_ == ci::osc::ArgType::STRING)
        {
            
            bone = message.getArgString(i);
            
            
            i++;
            bool end_val = false;
            std::vector<float> values;
            while (i<message.getNumArgs() && !end_val)
            {
                end_val = message.getArgType(i) != ci::osc::ArgType::FLOAT;
                if(!end_val)
                {
                    values.push_back(message.getArgFloat(i));
                }
                i++;
            }
            i-=2; //will always return 2 more than needed
            

            const int NUMBER_OFVALUES_NEEDED_TOBE_LIVE = 6; //just sayin'
            if(values.size() >= NUMBER_OFVALUES_NEEDED_TOBE_LIVE) //this is then, a measured value
            {
                createNotchMotionData(bone, sender, sendingDevice, values);
            }
            
            //if sending to wekinator
            //append to message and send to wek, if relevant
            if(SEND_TO_WEKINATOR && values.size() >= 6)
            {
                for(int i=0; i<values.size(); i++)
                    wekMsg.append(values[i]);
            }
        }


    }
    if(SEND_TO_WEKINATOR)
        mWekSender.send(wekMsg);
}

void FeverRhythmCycleMain::createNotchMotionData(std::string _id, std::string who, std::string sendingDevice, std::vector<float> vals)
{
    CRCPMotionAnalysis::MocapDeviceData *sensorData;
    
    sensorData = new CRCPMotionAnalysis::NotchDeviceData();
    sensorData->setSendingDevice(sendingDevice);
    
    //TODO: deal with the "who" value
    
    //hack hack -- change this value to indicate different people -- note: will need to add to OSC coming from phone - so let me know if anyone needs this
    const int notchDataID = 9;

    CRCPMotionAnalysis::SensorData *sensor = getSensor( _id, notchDataID, sensorData->getDeviceType(), sensorData->getSendingDevice() );
    
    //set time stamp
    sensorData->setData( CRCPMotionAnalysis::MocapDeviceData::DataIndices::TIME_STAMP, seconds ); //set timestamp from program -- synch with call to update()
    
    //add accel + bone position data
    for(int i= 0; i<3; i++){
        sensorData->setData(CRCPMotionAnalysis::MocapDeviceData::DataIndices::ACCELX+i, vals[i]);
        sensorData->setData(CRCPMotionAnalysis::MocapDeviceData::DataIndices::BONEANGLE_TILT+i, vals[i+3]);
        if(vals.size() > 6) //make compatible with previous recordings... I guess
        {
            sensorData->setData(CRCPMotionAnalysis::MocapDeviceData::DataIndices::RELATIVE_TILT+i, vals[i+6]);
            sensorData->setData(CRCPMotionAnalysis::MocapDeviceData::DataIndices::ANGVEL_TILT+i, vals[i+9]);
        }
        else
        {
//            std::cout << "Using depreciated apps & sensor recordings....\n";
        }
    }
    
    sensor->addSensorData(sensorData);
}

//set up osc
void FeverRhythmCycleMain::setup()
{
    testOSCNumber=0;

    printKeyboardMenu();
    
    mChangePeakThresholdMode = false;
    mWhichThresh = CRCPMotionAnalysis::BodyPartSensor::BodyPart::LEFTHAND;
    
    initCamera();
    
    try{
        mSender.bind();
    }
    catch( osc::Exception &e)
    {
        CI_LOG_E( "Error binding" << e.what() << " val: " << e.value() );
        quit();
    }
    
    try{
        mWekSender.bind();
    }
    catch( osc::Exception &e)
    {
        CI_LOG_E( "Error binding" << e.what() << " val: " << e.value() );
        quit();
    }
    
    try{
        mRemoteLaptopSender.bind();
    }
    catch( osc::Exception &e)
    {
        CI_LOG_E( "Error binding" << e.what() << " val: " << e.value() );
        quit();
    }
    
    //opens a file to save incoming OSC
    fs::path fpath = getSaveFilePath();
    saveOSC = new CRCPMotionAnalysis::SaveOSC(fpath.c_str());
    playOSC = NULL; //init the play to null
    
    //ListenerFn = std::function<void( const Message &message )>
    mReceiver.setListener( SYNTIEN_MESSAGE, [&]( const osc::Message &msg ){
        saveOSC->add(msg, getElapsedSeconds()); //add this line to save the OSC
        updatePhoneValues(msg);
    });
    
    for (int i=0; i<MAX_NUM_OF_WIIMOTES; i++)
    {
        std::stringstream addr;
        addr << WIIMOTE_ACCEL_MESSAGE_PART1 << i << WIIMOTE_ACCEL_MESSAGE_PART2;
        mReceiver.setListener( addr.str(), [&]( const osc::Message &msg ){
            saveOSC->add(msg, getElapsedSeconds()); //add this line to save the OSC
            updateWiiValues(msg);
        });
    }
    testOSCNumber = 0;
    mReceiver.setListener( NOTCH_MESSAGE, [&]( const osc::Message &msg ){
        saveOSC->add(msg, getElapsedSeconds()); //add this line to save the OSC
        updateNotchValues(msg);
    });
    
    mReceiver.setListener( "/testThisShit", [&]( const osc::Message &msg ){
        std::cout << "This shit works!!\n";
    });
    try {
        // Bind the receiver to the endpoint. This function may throw.
        mReceiver.bind();
    }
    catch( const osc::Exception &ex ) {
        CI_LOG_E( "Error binding: " << ex.what() << " val: " << ex.value() );
        quit();
    }
    
    // UDP opens the socket and "listens" accepting any message from any endpoint. The listen
    // function takes an error handler for the underlying socket. Any errors that would
    // call this function are because of problems with the socket or with the remote message.
    mReceiver.listen(
                     []( asio::error_code error, protocol::endpoint endpoint ) -> bool {
                         if( error ) {
                             CI_LOG_E( "Error Listening: " << error.message() << " val: " << error.value() << " endpoint: " << endpoint );
                             return false;
                         }
                         else
                             return true;
                     });
    
    
}

void FeverRhythmCycleMain::printKeyboardMenu()
{
    std::cout << "---------------------------------------------------------------------\n";
    std::cout << "                    Main Menu Instructions\n";
    std::cout << "---------------------------------------------------------------------\n";
    std::cout << " 1. Choose a file to save any incoming OSC or else click 'Cancel'  \n";
    std::cout << " 2. UI Menu:  \n";
    std::cout << " 'p' - Open a saved OSC file to play the saved stream. Timing is based on running time.  \n";
    std::cout << " Up & Down Mouse Drag  - Change y eye point in camera  \n";
    std::cout << " Left & Right Mouse Drag  - Change x eye point in camera \n";
    std::cout << " Shift key + Left & Right Mouse Drag  - Change z eye point in camera  \n";
    std::cout << " 'r' - Reset to default camera eyepoint coordinates in camera\n";
    std::cout << " 't' - Change to peak thresh mode\n";
    std::cout << " Peak thresh mode - '0'-'7' - Change bone\n";
    std::cout << " Peak thresh mode - Arrow up & down - adjust thresh\n";



    std::cout << "---------------------------------------------------------------------\n";
    std::cout << "---------------------------------------------------------------------\n";
    std::cout << "\n";
    std::cout << "\n";
}

void FeverRhythmCycleMain::keyDown( KeyEvent event )
{
    //change peak thresh mode
    if( mChangePeakThresholdMode)
    {
        std::string boneName = mPeople[0]->getBoneName((int) mWhichThresh);
        if(event.getCode() == KeyEvent::KEY_DOWN)
        {
            mPeople[0]->decreasePeakThresh(boneName);
        }
        else if (event.getCode() == KeyEvent::KEY_UP)
        {
            mPeople[0]->increasePeakThresh(boneName);
        }
        else
        {
            int num = event.getChar() -'0';
            if(num > 0 && num <= 7)
            {
                mWhichThresh = (CRCPMotionAnalysis::BodyPartSensor::BodyPart) num;
                boneName = mPeople[0]->getBoneName((int) mWhichThresh);
                std::cout << "Current Body Part to adjust peak thresh: " << boneName << std::endl;
                
            }
            std::cout << "Command not recognized in Change Peak Thresh Mode\n";
            
        }
    }
    
    
    //choose a file to play OSC, if wanted.
    if(event.getChar() == 'p')
    {
        fs::path filename = getOpenFilePath();
        playOSC = new CRCPMotionAnalysis::PlayOSC(filename.c_str(), DESTHOST, LOCALPORT2, LOCALPORT + 10);
    }
    if(event.getChar() == 'r')
    {
        std::cout << "Reset to default camera eyepoint coordinates\n";
        mEyePoint = mDefaultEyePoint;
    }
    else if(event.getChar() == 't')
    {
        if(mPeople.empty())
        {
            std::cout << "No skeletons sending data. Cannot enter changing thresh mode./n";
            return;
        }

        
        mChangePeakThresholdMode = !mChangePeakThresholdMode;
        if(mChangePeakThresholdMode)
        {
            std::cout << "Changing to peak threshold mode...\n";
            std::cout << "Currently changing thresh for " << mPeople[0]->getBoneName( (int) mWhichThresh );
        }
        else
        {
            std::cout << "Exiting peak threshold mode.\n";
        }
        

//        bool mChangePeakThresholdMode;
//        CRCPMotionAnalysis::BodyPartSensor::BodyPart mWhichThresh;
    }
    mShiftKeyDown = event.isShiftDown();
}

void FeverRhythmCycleMain::keyUp(KeyEvent event )
{
    mShiftKeyDown = event.isShiftDown();
}


//update entities and ugens and send OSC, if relevant
void FeverRhythmCycleMain::update()
{
//    std::cout << "updating" << ":" << testOSCNumber << std::endl;
//    std::cout.flush();
//    testOSCNumber++;
    
    updateCamera();
    mShiftKeyDown = false;
    seconds = getElapsedSeconds(); //clock the time update is called to sync incoming messages
    
    //update sensors
    for(int i=0; i<mSensors.size(); i++)
    {
        mSensors[i]->update(seconds);
    }
    //update all entities
    for(int i=0; i<mPeople.size(); i++)
    {
        mPeople[i]->update(seconds);
    }
    
    //send OSC from the entities -- after all are updated..
    for(int i=0; i<mPeople.size(); i++)
    {
        std::vector<osc::Message> msgs = mPeople[i]->getOSC();
        for(int i=0; i<msgs.size(); i++)
        {
            mSender.send(msgs[i]);
            mRemoteLaptopSender.send(msgs[i]); //also send to the remote laptop
        }
    }
    
    //update if playing from OSC saved to file
    if(playOSC != NULL)
    {
        playOSC->update(seconds);
    }
    
}

void FeverRhythmCycleMain::drawGrid(float size=100.0f, float step=2.0f)
{
    //draw 3d grid on the screen
    
    gl::color(Colorf(0.2f, 0.2f, 0.2f));
    for (float i = -size; i <= size; i += step)
    {
        gl::drawLine(vec3(i, 0.0f, -size), vec3(i, 0.0f, size));
        gl::drawLine(vec3(-size, 0.0f, i), vec3(size, 0.0f, i));
    }
}

void FeverRhythmCycleMain::mouseDown( MouseEvent event )
{
    mLastMouseEvent = event;
}

void FeverRhythmCycleMain::mouseDrag( MouseEvent event )
{
    ci::vec3 eyePointChange(0.0f, 0.0f, 0.0f);
    eyePointChange.x = (float(event.getX() - mLastMouseEvent.getX())/float(ci::app::getWindowWidth()));
    eyePointChange.y = (float(event.getY() - mLastMouseEvent.getY())/float(ci::app::getWindowHeight()));
    
//    std::cout << "eyePointChange: " << eyePointChange << std::endl;
    
    if(mShiftKeyDown)
    {
        mEyePoint.z+=eyePointChange.x;
    }
    else
        mEyePoint.x+=(eyePointChange.x);
    
    mEyePoint.y+=eyePointChange.y;

//    mLastMouseEvent = event;
}

void FeverRhythmCycleMain::initCamera()
{
    mEyePoint = mCamera.getEyePoint();
    mDefaultEyePoint = mEyePoint;
    mLookAt = ci::vec3(0,0,0); //mCamera.worldToScreen(<#const vec3 &worldCoord#>, <#float screenWidth#>, <#float screenHeight#>)
}

void FeverRhythmCycleMain::updateCamera()
{


    mCamera.setEyePoint( mEyePoint );
//    mCamera.setPerspective( mFov, mObjectFbo->getAspectRatio(), mNearPlane, mFarPlane );
//    mCamera.setLensShift( mLensShift );
    mCamera.lookAt( mLookAt );
//    mCamera.lookAt(vec3(150, 100, 100), vec3(0));
    gl::setMatrices( mCamera );
}



//draw the entities
void FeverRhythmCycleMain::draw()
{
    
    gl::clear( Color( 0, 0, 0 ) );
    
    gl::color(1, 1, 0, 1);
    ci::gl::setMatrices(mCamera);
    
    // enable the depth buffer (after all, we are doing 3D)
    gl::enableDepthRead();
    gl::enableDepthWrite();
    
    //    // draw the grid on the floor
    drawGrid();
    

    
    gl::color(1,0,1,1);
    ci::gl::drawSphere(ci::vec3(0, 0, 0), 0.05f);
    ci::gl::drawSphere(ci::vec3(0, 12, 0), 0.1f);
    ci::gl::drawSphere(ci::vec3(0, 0, 15), 0.1f);
    ci::gl::drawSphere(ci::vec3(15, 0, 0), 0.1f);
    gl::color(0,1,1,1);
    ci::gl::drawSphere(ci::vec3(0, -15, 0), 0.1f);
    ci::gl::drawSphere(ci::vec3(0, 0, -20), 0.1f);
    ci::gl::drawSphere(ci::vec3(-20, 0, 0), 0.1f);

//    auto lambert = gl::ShaderDef().lambert();
//    auto shader = gl::getStockShader( lambert );
//    shader->bind();
    
    for(int i=0; i<mPeople.size(); i++)
    {
        mPeople[i]->draw();
    }
    
}

CINDER_APP( FeverRhythmCycleMain, RendererGl,
           []( FeverRhythmCycleMain::Settings *settings ) //note: this part is to fix the display after updating OS X 1/15/18
           {
               settings->setHighDensityDisplayEnabled( true );
               settings->setTitle("Fever Rhythm Cycle");
               settings->setWindowSize(800, 600);

//               settings->setFrameRate(FRAMERATE); //set fastest framerate
           } )

