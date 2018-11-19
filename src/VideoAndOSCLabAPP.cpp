
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

#include "cinder/qtime/QuickTimeGl.h"


#include "Osc.h" //add to send OSC


#include "CinderOpenCV.h"

#define SAMPLE_WINDOW_MOD 300
#define MAX_FEATURES 300

#define LOCALPORT 8887
#define DESTPORT 8888
#define DESTHOST "127.0.0.1"

using namespace ci;
using namespace ci::app;
using namespace std;

class VideoAndOSCLabAPP : public App {
  public:
	void setup() override;
//    void mouseDown( MouseEvent event ) override;
    void keyDown( KeyEvent event ) override;

	void update() override;
	void draw() override;
    
    VideoAndOSCLabAPP(); //create to constructor init OSC
    
  protected:
    CaptureRef                 mCapture;
    gl::TextureRef             mTexture;
    
    SurfaceRef                 mSurface;
    
    void loadMovieFile( const fs::path &moviePath ); //https://libcinder.org/docs/classcinder_1_1qtime_1_1_movie_surface.html
    
    gl::TextureRef            mFrameTexture;
    qtime::MovieSurfaceRef        mMovie;
    
    osc::SenderUdp            mOscSender;
    
    void sendTimeViarOSC(float curtime);

    
};

VideoAndOSCLabAPP::VideoAndOSCLabAPP() :  mOscSender(LOCALPORT, DESTHOST, DESTPORT)
{
    //initialized sender
    
}

void VideoAndOSCLabAPP::setup()
{

    //MOVIE STUFF -- FIRST do getOpenFilePath(), THEN  getResource Path
    
    
//    fs::path moviePath = getOpenFilePath();
    fs::path moviePath = getResourcePath("BlackDot.mp4"); //for os x
    
    //myMovie = qtime::MovieGl::create( loadresource( "FlyingLogo.mov" ) ); //for
    //https://libcinder.org/docs/guides/resources/index.htmlwindows
    
    console() << "moviePath: " << moviePath << std::endl;

    if( ! moviePath.empty() )
        loadMovieFile( moviePath );
    
    //bind the sender to the ports
    try {
        mOscSender.bind();
    } catch (osc::Exception &e)
    {
        CI_LOG_E( "Error binding: " << e.what() << " val: " << e.value() );
        quit();
    }

}

void VideoAndOSCLabAPP::loadMovieFile( const fs::path &moviePath )
{
    try {
        // load up the movie, set it to loop, and begin playing
        mMovie = qtime::MovieSurface::create( moviePath );
        //mMovie->setLoop();
        mMovie->play();
        console() << "Playing: " << mMovie->isPlaying() << std::endl;
    }
    catch( ci::Exception &exc ) {
        console() << "Exception caught trying to load the movie from path: " << moviePath << ", what: " << exc.what() << std::endl;
        mMovie.reset();
    }
    
    mFrameTexture.reset();
}


void VideoAndOSCLabAPP::keyDown( KeyEvent event )
{
    if(event.getChar() == ' ')
    {
        mMovie->seekToStart();
        if(!mMovie->isPlaying()) mMovie->play();
    }
}

void VideoAndOSCLabAPP::update()
{
//    if(mCapture && mCapture->checkNewFrame()) //is there a new frame???? (& did camera get created?)
//    {
//        mSurface = mCapture->getSurface();
//
//        if(! mTexture)
//            mTexture = gl::Texture::create(*mSurface);
//        else
//            mTexture->update(*mSurface);
//    }
    
    if( mMovie )
    {
        mSurface = mMovie->getSurface();
        
        float curTime = mMovie->getCurrentTime(); //value to send osc
        sendTimeViarOSC(curTime);
        
    }
    
    if(mSurface)
        if(!mFrameTexture)
            mFrameTexture = gl::Texture::create(*mSurface);
        else { mFrameTexture->update(*mSurface); }
    
    static bool sPrintedDone = false;
    if( ! sPrintedDone && mMovie->isDone() ) {
        console() << "Done Playing" << std::endl;
        sPrintedDone = true;
    }
    
}

//send the time out to other applications
void VideoAndOSCLabAPP::sendTimeViarOSC(float curtime)
{
    osc::Message msg, msgPlaying;
    
    //set the  address - a string - highly rec. using a def or constant
    msg.setAddress("/MovieInfo/time");
    
    //add the data
    msg.append(curtime);
    
    //send
    mOscSender.send(msg); //can handle on onSendError
    
    msgPlaying.setAddress("/MovieInfo/playing");
    msgPlaying.append(mMovie->isPlaying());
    mOscSender.send(msgPlaying);
    
}


void VideoAndOSCLabAPP::draw()
{
    gl::clear( Color( 0, 0, 0 ) );

    gl::color( 1, 1, 1, 1 );

//    if( mTexture )
//    {
//        gl::draw( mTexture );
//    }

    
    if( mFrameTexture ) {
        Rectf centeredRect = Rectf( mFrameTexture->getBounds() ).getCenteredFit( getWindowBounds(), true );
    
        gl::draw( mFrameTexture, centeredRect );
    }
    
   

    //Thinking ahead: what things can you do?? -- track the motion direction? velocity, accel. etc.?
    
    
}

CINDER_APP( VideoAndOSCLabAPP, RendererGl )
