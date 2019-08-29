//
//  MocapCam.h
//  InteractiveTangoReadFromAndroid
//
//  Created by courtney on 9/16/14.
//
//

#ifndef InteractiveTangoReadFromAndroid_MocapCam_h
#define InteractiveTangoReadFromAndroid_MocapCam_h

class ReadMocapCameraFile
{
public:
	ReadMocapCameraFile(std::string fnamePic, std::string fnameRefs)
	{
		fname = fnamePic;
		fref = fnameRefs;
		ext = ".png";
		time_index = 0;
		createReferences();
		modified_time = 0;
        speed = 1;
	};
    inline void changeExt(std::string e){ ext = e; };
    inline void setSpeed(float s){ speed=s; };
    
    void createReferences()
    {
		
        std::ifstream fstr;
        
        std::string data;
        const int BUF_NUM = 1024 * 4;
        char str[BUF_NUM];
        int lastComma, fileIndex;
        
        fstr.open(fref, std::ios::in);
        if (fstr.is_open()) {
            
            while (!fstr.eof())// && lastComma!=-1
            {
                fstr.getline(str, BUF_NUM, '\n');
                data.assign(str);
                
                //do again if only these characters
                if (data.compare("\n") == 0 && data.compare("\r") == 0)
                {
                    fstr.getline(str, BUF_NUM);
                    data.assign(str);
                }
                
                lastComma = data.find_first_of(",");
                if (lastComma == -1)
                {
                    std::cout << "Incorrect Format\n";
                    return; //get out
                }
                fileIndex = lastComma;
                
//                int curFrame = atoi(data.substr(0, lastComma).c_str()); //use for debug, if needed
                float tt = atof(data.substr(lastComma + 1, data.length() - lastComma - 1).c_str());
                times.push_back(tt);
            }
        }
        fstr.close();
    }
    
    void update(float seconds)
    {
        std::stringstream ss;
        if (time_index < times.size())
			while (time_index < times.size() && times[time_index] < (seconds*speed+modified_time))
            {
                time_index++;
            }
        
        if (time_index>0) time_index--;
        
        ss << fname << time_index << ext;
        mTexture = ci::loadImage(ci::fs::path(ss.str()));
    };
    
    void draw()
    {
        ci::gl::draw(mTexture);
    }
    
    inline void setModifiedStart(float t)
    {
        modified_time = t;
    };
    
    void reset()
    {
        time_index = 0;
    };
private:
	std::string fname, fref;
	std::vector<float> times;
	int time_index;
	std::string ext;
	ci::gl::Texture mTexture;
	float modified_time;
    float speed;
};

class MoCapCamera
{
public:
	MoCapCamera();
	~MoCapCamera();
    
	void init(ci::Timer *t);
	void update();
	void draw();
	void setMarker();
    ci::gl::TextureRef getTexture();
    inline bool isInit(){ return mInit; };
private:
	ci::CaptureRef			mCapture;
	ci::gl::TextureRef		mTexture, mTimeTexture;
	ci::Timer				*mTimer;
	int						markerNum;
	bool					showMarker;
    bool                    mInit;
};

MoCapCamera::MoCapCamera()
{
	markerNum = 0;
	showMarker = false;
    mInit = false;
}

MoCapCamera::~MoCapCamera()
{
}

void MoCapCamera::init(ci::Timer *t)
{
	mTimer = t;
	try {
//		mCapture = ci::Capture::create(480, 360);
        std::vector< ci::Capture::DeviceRef > devices =  ci::Capture::getDevices();
        
        if(devices.size() > 1)
        {
            mCapture =   ci::Capture::create( 480, 360, devices[1] );
        }
        else mCapture = ci::Capture::create(480, 360);
            
        mCapture->start();
        
        mInit = true;

	}
	catch (...) {
		ci::app::console() << "Failed to initialize capture" << std::endl;
	}
}

void MoCapCamera::setMarker()
{
	showMarker = true;
}

void MoCapCamera::update()
{
	if (mCapture && mCapture->checkNewFrame()) {
		ci::Surface movImg = mCapture->getSurface();
        //		ci::Surface smImg(320 / 2, 240 / 2, false, ci::SurfaceChannelOrder::RGBA );
        //		ci::ip::resize(movImg, movImg.getBounds(), &smImg, ci::Area(0, 0, 320/2, 240/2), ci::FilterSincBlackman());
		mTexture = ci::gl::Texture::create(movImg);
        
		if (mTimer != NULL)
		{
			ci::TextLayout layout;
			std::stringstream ss;
            
			ss << mTimer->getSeconds();
            
			if (showMarker)
			{
				ss << "      MARKER: " << markerNum;
				showMarker = false;
				markerNum++;
			}
            
			layout.setFont(ci::Font("Arial", 16));
			layout.setColor(ci::ColorA(1, 1, 1, 1));
			layout.addLine(ss.str());
			mTimeTexture = ci::gl::Texture::create(layout.render(true));
		}
	}
}

void MoCapCamera::draw()
{
	if (mTexture) 
	{
		ci::Vec2i loc((640.0 / 2.5), (480.0 / 2.5));
        
		//glPushMatrix();
		ci::gl::draw(mTexture, loc);
		//glPopMatrix();
        
		if (mTimeTexture)
		{
			ci::gl::enableAlphaBlending();
			loc.y -= 15; 
			ci::gl::draw(mTimeTexture, loc);
			std::cout << mTimer->getSeconds() << "";
			ci::gl::disableAlphaBlending();
		}
	}
}


ci::gl::TextureRef MoCapCamera::getTexture()
{
    return mTexture;
}


#endif
