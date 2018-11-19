
//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

//includes for background subtraction
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>


#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Capture.h" //needed for capture
#include "cinder/Log.h" //needed to log errors

#include <sstream>

#include "CinderOpenCV.h"

#include "Osc.h"

#include "MotionCaptureData.h"
#include "Sensor.h"
#include "UGENs.h"
#include "PeakDetection.h"
#include "MeasuredEntities.h"
#include "SaveOSC.h"

#define LOCALPORT 8886
#define LOCALPORT2 8887
#define DESTHOST "127.0.0.1"
#define DESTPORT 8888

//osc messages
#define ELAPSED_FRAMES_ADDR "/VideoAndOSCLab/elapsedFrames"
#define ELAPSED_SECS_ADDR "/VideoAndOSCLab/elapsedSeconds"
#define SYNTIEN_MESSAGE "/syntien/motion/1/scope1"
#define WIIMOTE_ACCEL_MESSAGE_PART1 "/wii/"
#define WIIMOTE_ACCEL_MESSAGE_PART2 "/accel/pry"
#define WIIMOTE_BUTTON_1 "/wii/1/button/1"
#define NOTCH_MESSAGE "/Notch/BonePosAndAccel"

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
class MotionSensorSignalTreeExample : public App {
public:
    MotionSensorSignalTreeExample();
    
    void setup() override;
    //    void mouseDown( MouseEvent event ) override;
    void keyDown( KeyEvent event ) override;
    
    void update() override;
    void draw() override;
    
protected:
    CaptureRef                 mCapture;
    gl::TextureRef             mTexture;
    SurfaceRef                 mSurface;
    
    osc::SenderUdp             mSender;
    
    void sendOSC(std::string addr, float value);
    
    Receiver mReceiver; //new
    std::map<uint64_t, protocol::endpoint> mConnections; //new
    
    //below for printing out iphone info
    std::vector<ci::vec2> points;
    std::vector<float>  alpha;
    
    void updatePhoneValues(const osc::Message &message);
    void updateWiiValues(const osc::Message &message);
    void updateNotchValues(const osc::Message &message);
    void createNotchMotionData(std::string _id, std::vector<float> vals);
    void printNotchValues(const osc::Message &message);
    void addPhoneAndWiiData(const osc::Message &message, std::string _id);
    
    CRCPMotionAnalysis::SensorData *getSensor( std::string _id, int which, CRCPMotionAnalysis::MocapDeviceData::MocapDevice device ); // find sensor or wiimote in list via id
    std::vector<CRCPMotionAnalysis::SensorData *> mSensors; //all the sensors which have sent us OSC -- well only wiimotes so far
    std::vector<CRCPMotionAnalysis::Entity *> mEntities;  //who are we measuring? change name when specifics are known.
    
    float seconds; //where we are
    
    //file
    CRCPMotionAnalysis::SaveOSC *saveOSC;
    CRCPMotionAnalysis::PlayOSC *playOSC;
};

MotionSensorSignalTreeExample::MotionSensorSignalTreeExample() : mSender(LOCALPORT, DESTHOST, DESTPORT), mReceiver( LOCALPORT2 )
{
    
}

//outdated vestige
void MotionSensorSignalTreeExample::sendOSC(std::string addr, float value)
{
    osc::Message msg;
    msg.setAddress(addr);
    msg.append(value); //adds a parameter
    mSender.send(msg);
}

//this has not been implemented into signal tree paradigm yet. ah well. TODO: implement as such
void MotionSensorSignalTreeExample::updatePhoneValues(const osc::Message &message)
{
//    std::cout << "received\n";
    addPhoneAndWiiData(message, PHONE_ID);
}

//gets data from osc message then adds wiimote data to sensors
void MotionSensorSignalTreeExample::addPhoneAndWiiData(const osc::Message &message, std::string _id)
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

    
    CRCPMotionAnalysis::SensorData *sensor = getSensor( _id, which, sensorData->getDeviceType() );
    
    //set time stamp
    sensorData->setData( CRCPMotionAnalysis::MocapDeviceData::DataIndices::TIME_STAMP, seconds ); //set timestamp from program -- synch with call to update()
    
    //add accel data
    for(int i= 0; i<3; i++)
        sensorData->setData(CRCPMotionAnalysis::MocapDeviceData::DataIndices::ACCELX+i, message.getArgFloat(i));
    
    sensor->addSensorData(sensorData);
}

//return sensor with id & or create one w/detected id then return that one
CRCPMotionAnalysis::SensorData *MotionSensorSignalTreeExample::getSensor( std::string _id, int which, CRCPMotionAnalysis::MocapDeviceData::MocapDevice device )
{
    
    bool found = false;
    int index = 0;
    
    while( !found && index < mSensors.size() )
    {
        found = mSensors[index]->same( _id, which );
        index++;
    }
    
    if(found)
    {
        return mSensors[index-1];
    }
    else
    {
        CRCPMotionAnalysis::SensorData *sensor = new CRCPMotionAnalysis::SensorData( _id, which, device );
        mSensors.push_back(sensor);
            
        //add to 'entity' the data structure which can combine sensors. It currently only has one body part so it is simple.
        int entityID  = mSensors.size()-1;
        CRCPMotionAnalysis::Entity *entity = new CRCPMotionAnalysis::Entity();
        entity->addSensorBodyPart(entityID, sensor, CRCPMotionAnalysis::Entity::BodyPart::HAND );  //note that this should change if using bones, etc.
        mEntities.push_back(entity);
        return sensor;
    }
}

//finds the id of the wiimote then adds the wiidata to ugens
void MotionSensorSignalTreeExample::updateWiiValues(const osc::Message &message)
{
    //get which wii
    std::string addr = message.getAddress();
    std::string pt1 = WIIMOTE_ACCEL_MESSAGE_PART1;
    int index = addr.find_first_of(pt1);
    std::string whichWii = addr.substr(index+pt1.length(), 1);
    addPhoneAndWiiData(message, whichWii);
}

void MotionSensorSignalTreeExample::printNotchValues(const osc::Message &message)
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
    std::cout << std::endl;

}

//handles notch data
void MotionSensorSignalTreeExample::updateNotchValues(const osc::Message &message)
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
     
     ***/
    
    std::string bone = "";

    for ( int i=2; i<message.getNumArgs(); i++)
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
            
            if(values.size() >= 6) //this is then, a measured value
            {
                createNotchMotionData(bone, values);
            }
        }
        
    }
}

void MotionSensorSignalTreeExample::createNotchMotionData(std::string _id, std::vector<float> vals)
{
    CRCPMotionAnalysis::MocapDeviceData *sensorData;
    
    sensorData = new CRCPMotionAnalysis::NotchDeviceData();
    
    //hack hack -- change this value to indicate different people -- note: will need to add to OSC coming from phone - so let me know if anyone needs this
    const int notchDataID = 9;

    CRCPMotionAnalysis::SensorData *sensor = getSensor( _id, notchDataID, sensorData->getDeviceType() );
    
    //set time stamp
    sensorData->setData( CRCPMotionAnalysis::MocapDeviceData::DataIndices::TIME_STAMP, seconds ); //set timestamp from program -- synch with call to update()
    
    //add accel + bone position data
    for(int i= 0; i<3; i++){
        sensorData->setData(CRCPMotionAnalysis::MocapDeviceData::DataIndices::ACCELX+i, vals[i]);
        sensorData->setData(CRCPMotionAnalysis::MocapDeviceData::DataIndices::BONEANGLE_TILT+i, vals[i+3]);
    }
    
    sensor->addSensorData(sensorData);
}

//set up osc
void MotionSensorSignalTreeExample::setup()
{
    try{
        mSender.bind();
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
    
    mReceiver.setListener( NOTCH_MESSAGE, [&]( const osc::Message &msg ){
        saveOSC->add(msg, getElapsedSeconds()); //add this line to save the OSC
        updateNotchValues(msg);
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



void MotionSensorSignalTreeExample::keyDown( KeyEvent event )
{
    //choose a file to play OSC, if wanted.
    if(event.getChar() == 'o')
    {
        fs::path filename = getOpenFilePath();
        playOSC = new CRCPMotionAnalysis::PlayOSC(filename.c_str(), DESTHOST, LOCALPORT2, LOCALPORT + 10);
    }
}

//update entities and ugens and send OSC, if relevant
void MotionSensorSignalTreeExample::update()
{
    seconds = getElapsedSeconds(); //clock the time update is called to sync incoming messages
    
    //update sensors
    for(int i=0; i<mSensors.size(); i++)
    {
        mSensors[i]->update(seconds);
    }
    //update all entities
    for(int i=0; i<mEntities.size(); i++)
    {
        mEntities[i]->update(seconds);
    }
    
    //send OSC from the entities -- after all are updated..
    for(int i=0; i<mEntities.size(); i++)
    {
        std::vector<osc::Message> msgs = mEntities[i]->getOSC();
        for(int i=0; i<msgs.size(); i++)
        {
            mSender.send(msgs[i]);
        }
    }
    
    //update if playing from OSC saved to file
    if(playOSC != NULL)
    {
        playOSC->update(seconds);
    }
    
}

//draw the entities
void MotionSensorSignalTreeExample::draw()
{
    
    gl::clear( Color( 0, 0, 0 ) );
    
    for(int i=0; i<mEntities.size(); i++)
    {
        mEntities[i]->draw();
    }
    
}

CINDER_APP( MotionSensorSignalTreeExample, RendererGl )

