//
//  UGENs.h
//
//  Created by Dr. Courtney Brown, 2014-2018
//  A set of unit generators/filters and a framework for use in motion capture class
//  These ugens are for retrieving motion data features and signal conditioning.
//  Modified 10-22-2018 for utility of mocap class.
//

#ifndef UGENs_h
#define UGENs_h

namespace CRCPMotionAnalysis {
    
    static const double SR = 50; // Sample rate -- this may vary for sensors...
    
    
    //abstract class of all ugens
    class UGEN
    {
    public:
        UGEN(){};
        virtual std::vector<ci::osc::Message> getOSC()=0;//<-- create/collect OSC messages that you may want to send to another program or computer
        virtual void update(float seconds=0)= 0; //<-- do the meat of the signal processing / feature extraction here
    };
    
    class SignalAnalysis : public UGEN
    {
    protected:
        std::vector<MocapDeviceData *> data1;
        std::vector<MocapDeviceData *> data2;
        
        SignalAnalysis *ugen, *ugen2  ;
        int buffersize;
        
        bool useAccel;
        bool useGry;
        bool useQuart;
        
        //find the average over a window given an input & start & index of a buffer
        virtual double findAvg(std::vector<double> input, int start, int end)
        {
            double N = end - start;
            double sum = 0;
            for(int i=start; i<end; i++)
            {
                sum += input[i];
            }
            return (sum/N);
        };
        
    public:
        //create analysis with pointers to prev. signal analyses which provide input data + how much data is in the buffer
        //note:  this takes  data from maximum 2 other signal analyses -- if you anticipated more, would want to redesign std::vector<SignalAlysis *> for max flexibility
        SignalAnalysis(SignalAnalysis *s1 = NULL, int bufsize=48, SignalAnalysis *s2 = NULL)
        {
            ugen = s1;
            ugen2 = s2;
            setBufferSize(bufsize);
            
            useAccel=true;
            useGry=false;
            useQuart=false;
            
        };
        
        //how many new samples do we need to process
        inline virtual int getNewSampleCount()
        {
            if(ugen != NULL && ugen2==NULL)
                return ugen->getNewSampleCount();
            else if(ugen != NULL && ugen2!=NULL)
                return std::max(ugen->getNewSampleCount(), ugen2->getNewSampleCount());
            else return 0;
        }
        
        //which signals are we dealing with or using?
        inline void processAccel(bool a)
        {
            useAccel = a;
        };
        
        inline void processGry(bool g)
        {
            useGry = g;
        };
        
        inline void processQuart(bool q)
        {
            useQuart=q;
        };
        
        inline virtual void setBufferSize(int sz)
        {
            buffersize = sz;
        };
        
        virtual inline std::vector<MocapDeviceData *> getBuffer(){
            return data1;
        };
        
        virtual inline std::vector<MocapDeviceData *> getBuffer2(){
            return data2;
        };
        
        int getBufferSize(){return buffersize;};
        
        virtual void update(float seconds=0)
        { 
            if( ugen != NULL ) data1 = ugen->getBuffer();
            if( ugen2 != NULL ) data2 = ugen2->getBuffer();
        };
        
#ifdef USING_ITPP //if we are using the it++ library which allows some signal processing + matlab functionalty then compile this wrapper/conversion function --> note: we're not.. as of yet.
        std::vector<itpp::vec> toITPPVector(std::vector<MocapDeviceData *> sdata) //to the it++ vector data type
        {
            std::vector<itpp::vec> newVec;
            for(int i=0; i<20; i++)
            {
                itpp::vec v(sdata.size());
                for(int j=0; j<sdata.size(); j++)
                {
                    v.set( j, sdata[j]->getData(i));
                }
                newVec.push_back(v);
            }
            return newVec;
        };
        std::vector<float> toFloatVector(itpp::vec input) //from the it++ vector data type
        {
            std::vector<float> output;
            for(int i=0; i<input.length(); i++)
            {
                output.push_back(input[i]);
            }
            return output;
        };
#endif

        //finds average in buffer using the MotionCaptureData instead of float vector as before
        virtual double findAvg(std::vector<MocapDeviceData *> input, int start, int end, int index)
        {
            double N = end - start;
            double sum = 0;
            for(int i=start; i<end; i++)
            {
                sum += input[i]->getData(index);
            }
            return (sum/N);
        };
    };


//This ugen only gets/receives inputs from sensors so sends nothing -- not an output bc it doesn't modify its inputs
//TODO: this filters for acc data ONLY, so turn on options, etc. if needed
class InputSignal : public SignalAnalysis
{
protected:
    int ID1;
    bool isPhone;
    bool isWiiMote;
    SensorData *sensor  ;

public:
    InputSignal(int idz, bool phone=false, SignalAnalysis *s1 = NULL, int bufsize=48, SignalAnalysis *s2 = NULL) : SignalAnalysis(s1, bufsize, s2)
    {
        ID1= idz;
        isPhone = phone;
    };
    
    //not sending any OSC currently
    virtual std::vector<ci::osc::Message> getOSC()
    {
        std::vector<ci::osc::Message> msgs;
        
        return msgs;
    };
    
    void setInput( SensorData *s1 )
    {
        sensor = s1;
    };
    
    virtual int getNewSampleCount()
    {
        return sensor->getNewSampleCount();
    };
    
    //puts valid mocap data in buffers for other ugens.
    virtual void update(float seconds=0)
    {
        data1.clear();
        if( sensor != NULL )
        {
            std::vector<MocapDeviceData *> data = sensor->getBuffer(buffersize);
            for(int i =0; i<data.size(); i++) //filters out dummy data
            {
                if( data[i]->getData(MocapDeviceData::DataIndices::ACCELX) != NO_DATA )
                {
                    data1.push_back(data[i]);
                 }
            }
        }
    };
    
    virtual std::vector<MocapDeviceData *> getBuffer(){
        return data1;
    };
    
};

    //has output signals that it modifies  & forwards on to others
    //only handles one stream of data...
    class OutputSignalAnalysis : public SignalAnalysis
    {
    protected:
        std::vector<MocapDeviceData *> outdata1;
        bool _sendOSC;
        int _id;
        std::string whichBodyPart;

        
        void eraseData()
        {
            for( int i=0; i<outdata1.size(); i++ )
                delete outdata1[i];
            
            outdata1.clear();
        };
    public:
        
        OutputSignalAnalysis(SignalAnalysis *s1, int bufsize, int sensorID=0, std::string whichPart="", bool sendOSC=false, SignalAnalysis *s2 = NULL) : SignalAnalysis(s1, bufsize, s2)
        {
            whichBodyPart = whichPart;
            _id=sensorID;
            _sendOSC = sendOSC;
        }
        
        //puts in accel data slots -- all other data left alone -- ALSO only
        void toOutputVector( std::vector<float> inputX, std::vector<float> inputY, std::vector<float> inputZ )
        {
            for(int i=0; i<inputX.size(); i++)
            {
                MocapDeviceData *data = new MocapDeviceData();
                data->setData(MocapDeviceData::DataIndices::INDEX, data1[i]->getData(MocapDeviceData::DataIndices::INDEX));
                data->setData(MocapDeviceData::DataIndices::TIME_STAMP, data1[i]->getData(MocapDeviceData::DataIndices::TIME_STAMP));
                data->setData(MocapDeviceData::DataIndices::ACCELX, inputX[i]);
                data->setData(MocapDeviceData::DataIndices::ACCELY, inputY[i]);
                data->setData(MocapDeviceData::DataIndices::ACCELZ, inputZ[i]);
                outdata1.push_back(data);
            }
        };
        
        virtual void update(float seconds = 0){
            eraseData();
            SignalAnalysis::update(seconds);
        };
        
        //just give it your data
        virtual std::vector<MocapDeviceData *> getBuffer(){
            return outdata1;
        };
    
    };
    
    //this class averages over a window - it is -- ***this only operates on accel. data***
    //--> but it can v. easily be expanded to include other data -- see commented out functionality
    class AveragingFilter : public OutputSignalAnalysis
    {
    protected:
        int windowSize;
    public:
        
        AveragingFilter(SignalAnalysis *s1, int w=10, int bufsize=16, int sensorID=0, std::string whichPart="", bool sendOSC=false ) : OutputSignalAnalysis(s1, bufsize, sensorID, whichPart, sendOSC)
        {
            windowSize = w;
        };
        
        //I'm gonna be shot for yet another avg function
        float mocapDeviceAvg(std::vector<MocapDeviceData *> data, int start, int end, int index )
        {
            double sum = 0;
            int valCount = 0;
            for( int j=start; j<=end; j++ )
            {
                if(data1[j]->getData(index) != NO_DATA)
                {
                    sum += data1[j]->getData(index);
                    valCount++;
                }
            }
            if(valCount==0) return NO_DATA;
            else return sum / double( valCount );
        };
        
        //perform the averaging here...
        virtual void update(float seconds=0)
        {
            OutputSignalAnalysis::update(seconds);
            if( data1.size() < buffersize) return ;
            
            for( int i=0; i<data1.size(); i++ )
            {
                int start = std::max(0, i-windowSize);
                int end = i;
                MocapDeviceData *mdd= new MocapDeviceData();
                mdd->setData(MocapDeviceData::DataIndices::INDEX, data1[i]->getData(MocapDeviceData::DataIndices::INDEX));
                mdd->setData(MocapDeviceData::DataIndices::TIME_STAMP, data1[i]->getData(MocapDeviceData::DataIndices::TIME_STAMP));
                
                if( useAccel )
                {
                    mdd->setData(MocapDeviceData::DataIndices::ACCELX, mocapDeviceAvg(data1, start, end, MocapDeviceData::DataIndices::ACCELX));
                    mdd->setData(MocapDeviceData::DataIndices::ACCELY, mocapDeviceAvg(data1, start, end, MocapDeviceData::DataIndices::ACCELY));
                    mdd->setData(MocapDeviceData::DataIndices::ACCELZ, mocapDeviceAvg(data1, start, end, MocapDeviceData::DataIndices::ACCELZ));
                }
                
                for(int j= MocapDeviceData::DataIndices::BONEANGLE_TILT; j<=MocapDeviceData::DataIndices::BONEANGLE_TILT+2; j++)
                {
                    mdd->setData(j, mocapDeviceAvg(data1, start, end, j));
                }
                
                //derivative of bone angles
                for(int j= MocapDeviceData::DataIndices::RELATIVE_TILT; j<=MocapDeviceData::DataIndices::RELATIVE_LATERAL; j++)
                {
                    mdd->setData(j, mocapDeviceAvg(data1, start, end, j));
                }


      
//    untested functionality in this domain...
//                if( useGry )
//                {
//                    mdd->setData(MotionDeviceData::DataIndices::GYROX, shimmerAvg(data1, start, end, ShimmerData::DataIndices::GYROX));
//                    mdd->setData(MotionDeviceData::DataIndices::GYROY, shimmerAvg(data1, start, end, ShimmerData::DataIndices::GYROY));
//                    mdd->setData(MotionDeviceData::DataIndices::GYROZ, shimmerAvg(data1, start, end, ShimmerData::DataIndices::GYROZ));
//                }
//
//                if( useQuart )
//                {
//                    mdd->setData(MotionDeviceData::DataIndices::QX, shimmerAvg(data1, start, end, ShimmerData::DataIndices::QX));
//                    mdd->setData(MotionDeviceData::DataIndices::QY, shimmerAvg(data1, start, end, ShimmerData::DataIndices::QY));
//                    mdd->setData(MotionDeviceData::DataIndices::QZ, shimmerAvg(data1, start, end, ShimmerData::DataIndices::QZ));
//                    mdd->setData(MotionDeviceData::DataIndices::QA, shimmerAvg(data1, start, end, ShimmerData::DataIndices::QA));
//                }
                outdata1.push_back(mdd);
            }
        }
        
        //if you wanted to send the signal somewhere
        std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            
            
            if(_sendOSC)
            {
                for( int i=0; i<outdata1.size(); i++ )
                {
                    
                    ci::osc::Message msg;
                    msg.setAddress(SIGAVG_OSCMESSAGE);
//                    std::cout << msg.getAddress() << "   ";
                    
                    msg.append(int(_id));
//                    std::cout << msg.getArgInt32(0) << ",";
                    
//                    std::cout << whichBodyPart << ",";
                    
                    msg.append(double(outdata1[i]->getData(MocapDeviceData::DataIndices::INDEX)));
//                    std::cout << msg.getArgDouble(1) << ",";
                    
                    msg.append(float(outdata1[i]->getData(MocapDeviceData::DataIndices::TIME_STAMP)));
//                    std::cout << msg.getArgFloat(2) << ",";
                    
                    
                    if( useAccel )
                    {
                        int h = 3;
                        for(int j= MocapDeviceData::DataIndices::ACCELX; j<=MocapDeviceData::DataIndices::ACCELZ; j++)
                        {
                            msg.append(float(outdata1[i]->getData(j)));
//                            std::cout << msg.getArgFloat(h) << ",";
                            h++;
                            
                        }
                    }
                    
                    //derivative of bone angles
                    for(int j= MocapDeviceData::DataIndices::BONEANGLE_TILT; j<=MocapDeviceData::DataIndices::BONEANGLE_LATERAL; j++)
                    {
                        msg.append(float(outdata1[i]->getData(j)));
//                        std::cout << outdata1[i]->getData(j) << ",";
                        
                    }
                    
                    //derivative of bone angles
                    for(int j= MocapDeviceData::DataIndices::RELATIVE_TILT; j<=MocapDeviceData::DataIndices::RELATIVE_LATERAL; j++)
                    {
                        msg.append(float(outdata1[i]->getData(j)));
//                        std::cout << outdata1[i]->getData(j) << ",";
                        
                    }
//                    std::cout << std::endl;
                    
//                    std::cout << msg << std::endl;
                    msgs.push_back(msg);
                    
                }
            }
            return msgs;
        };

    };
    
    //finds the derivative of the data
    class Derivative : public OutputSignalAnalysis
    {
    public:

        Derivative(SignalAnalysis *s1, int bufsize, int sensorID=0, std::string whichPart="", bool sendOSC=false) : OutputSignalAnalysis(s1, bufsize, sensorID, whichPart, sendOSC)
        {
//            _sendOSC = sendOSC;
            useAccel = true;
//            _id = sensorID;
//            whichBodyPart = whichPart;
        };
        
        //perform the derivative here...
        virtual void update(float seconds=0)
        {
            OutputSignalAnalysis::update(seconds);
            if( data1.size() < buffersize) return ;
            
            for( int i=1; i<data1.size(); i++ )
            {
                MocapDeviceData *mdd= new MocapDeviceData();
                mdd->setData(MocapDeviceData::DataIndices::INDEX, data1[i]->getData(MocapDeviceData::DataIndices::INDEX));
                mdd->setData(MocapDeviceData::DataIndices::TIME_STAMP, data1[i]->getData(MocapDeviceData::DataIndices::TIME_STAMP));
                
                if( useAccel )
                {
                    for(int j= MocapDeviceData::DataIndices::ACCELX; j<=MocapDeviceData::DataIndices::ACCELZ; j++)
                        mdd->setData(j, data1[i]->getData(j)-data1[i-1]->getData(j));
                }
                
                //derivative of bone angles
                for(int j= MocapDeviceData::DataIndices::BONEANGLE_TILT; j<=MocapDeviceData::DataIndices::BONEANGLE_TILT+2; j++)
                {
                    if(data1[i]->getData(j)!=NO_DATA)
                        mdd->setData(j, data1[i]->getData(j)-data1[i-1]->getData(j));
                }
                
                //relative angle to parent bone
                for(int j= MocapDeviceData::DataIndices::RELATIVE_TILT; j<=MocapDeviceData::DataIndices::RELATIVE_LATERAL; j++)
                {
                    if(data1[i]->getData(j)!=NO_DATA)
                        mdd->setData(j, data1[i]->getData(j)-data1[i-1]->getData(j));
                }
                
                //    untested functionality in this domain...
//                                if( useGry )
//                                {
//                                        for(int j= MocapDeviceData::DataIndices::GRYX; j<MocapDeviceData::DataIndices::GRYZ; j++)
//                                            mdd->setData(j, data1[i]->getData(j)-data1[i-1]->getData(j));
//                                        }
//
//                                if( useQuart )
//                                {
//                                        for(int j= MotionDeviceData::DataIndices::QX; j<MotionDeviceData::DataIndices::QA; j++){
//                                            mdd->setData(j, data1[i]->getData(j)-data1[i-1]->getData(j));
//                                        }
//                                }
                outdata1.push_back(mdd);
            }
        }

        //if you wanted to send the signal somewhere
        std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;

            
            if(_sendOSC)
            {
                for( int i=0; i<outdata1.size(); i++ )
                {
                    
                    ci::osc::Message msg;
                    msg.setAddress(DERIVATIVE_OSCMESSAGE);
//                    std::cout << msg.getAddress() << "   ";

                    msg.append(int(_id));
//                    std::cout << msg.getArgInt32(0) << ",";
                    
//                    std::cout << whichBodyPart << ",";
                
                    msg.append(double(outdata1[i]->getData(MocapDeviceData::DataIndices::INDEX)));
//                    std::cout << msg.getArgDouble(1) << ",";
                    
                    msg.append(float(outdata1[i]->getData(MocapDeviceData::DataIndices::TIME_STAMP)));
//                    std::cout << msg.getArgFloat(2) << ",";

                
                    if( useAccel )
                    {
                        int h = 3;
                        for(int j= MocapDeviceData::DataIndices::ACCELX; j<=MocapDeviceData::DataIndices::ACCELZ; j++)
                        {
                            msg.append(float(outdata1[i]->getData(j)));
//                            std::cout << msg.getArgFloat(h) << ",";
                            h++;

                        }
                    }
                
                    //derivative of bone positions
                    for(int j= MocapDeviceData::DataIndices::BONEANGLE_TILT; j<=MocapDeviceData::DataIndices::BONEANGLE_LATERAL; j++)
                    {
                        msg.append(float(outdata1[i]->getData(j)));
//                        std::cout << outdata1[i]->getData(j) << ",";

                    }
//
                    //relative angle to parent bone
                    for(int j= MocapDeviceData::DataIndices::RELATIVE_TILT; j<=MocapDeviceData::DataIndices::RELATIVE_LATERAL; j++)
                    {
                        msg.append(float(outdata1[i]->getData(j)));
                        //                        std::cout << outdata1[i]->getData(j) << ",";
                        
                    }
                    
                    
//                    std::cout << std::endl;
                    
                    msgs.push_back(msg);
                    
                }
            }
            return msgs;
        };
    };
    
    //this class visualizes incoming motion data
    class MocapDataVisualizer : public SignalAnalysis
    {
    protected :
       int  maxDraw; //how far back in history to draw
        
        std::vector<ci::vec2> points;
        std::vector<float>  alpha;
    public:
        MocapDataVisualizer(OutputSignalAnalysis *s1 = NULL, int _maxDraw=25,int bufsize=48, SignalAnalysis *s2 = NULL) : SignalAnalysis(s1, bufsize, s2)
        {
            maxDraw = _maxDraw;
        };
        
        //add data as screen positions and color alphas.
        virtual void update(float seconds = 0)
        {
            std::vector<MocapDeviceData *> buffer = ugen->getBuffer();
            if(buffer.size()<maxDraw) return; //ah well I don't want to handle smaller buffer sizes for this function. feel free to implement that.
            
            points.clear();
            alpha.clear();
            for(int i=buffer.size()-maxDraw; i<buffer.size(); i++)
            {
                MocapDeviceData *sample = buffer[i];
                sample->scaleAccel(); //0..1
                
//                std::cout << sample->toString() << std::endl;
                
                //ok now to screens
                points.push_back(ci::vec2((sample->getData(MocapDeviceData::DataIndices::ACCELX)*ci::app::getWindowWidth()/2.0) + ci::app::getWindowWidth()/4.0, sample->getData(MocapDeviceData::DataIndices::ACCELY)*ci::app::getWindowHeight()));
                alpha.push_back(sample->getData(MocapDeviceData::DataIndices::ACCELZ));
            }
        };
        
        //visualize the data
        virtual void draw()
        {
            float circleSize = 2;
            for(int i=1; i<points.size(); i++)
            {
                ci::gl::color(1.0f, 1.0f/alpha[i], alpha[i], 1.0f); //alpha is no longer alpha hmm

                ci::gl::drawLine(points[i-1], points[i]);
                ci::gl::drawSolidCircle(points[i], circleSize);
                
            }
            

        };
        
        
        //if you wanted to send something somewhere... prob. not
        std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            return msgs;
        };


};
    
class MocapDataVisualizerNotchBonePosition : public MocapDataVisualizer
{
public:
    MocapDataVisualizerNotchBonePosition(OutputSignalAnalysis *s1, int _maxDraw=25,int bufsize=48, SignalAnalysis *s2 = NULL)  : MocapDataVisualizer(s1, _maxDraw, bufsize, s2)
    {
        
    }
    
    //add data as screen positions and color alphas instead of drawing accel, will draw BONEANGLE_TILT=14, BONEANGLE_ROTATE=15, BONEANGLE_LATERAL=16
    //not the best visualization -- you are welcome to change  & make 3d -- but I think  the visualizer on the phone should generally give you an idea
    //of what is happening.
    virtual void update(float seconds = 0)
    {
        std::vector<MocapDeviceData *> buffer = ugen->getBuffer();
        if(buffer.size()<maxDraw) return; //ah well I don't want to handle smaller buffer sizes for this function. feel free to implement that.
        
        points.clear();
        alpha.clear();
        for(int i=buffer.size()-maxDraw; i<buffer.size(); i++)
        {
            MocapDeviceData *sample = buffer[i];
            sample->scaleAccel(); //0..1  to 0 to
            
//           std::cout << sample->toString() << std::endl;
            
            //ok now to screens
//            points.push_back(ci::vec2((sample->getData(MocapDeviceData::DataIndices::BONEANGLE_TILT)*ci::app::getWindowWidth()/2)+ci::app::getWindowWidth()/2, (sample->getData(MocapDeviceData::DataIndices::BONEANGLE_ROTATE)*ci::app::getWindowHeight())+ci::app::getWindowHeight()/2));
            points.push_back(ci::vec2((sample->getData(MocapDeviceData::DataIndices::BONEANGLE_TILT)*ci::app::getWindowWidth()/2)+ci::app::getWindowWidth()/4, (sample->getData(MocapDeviceData::DataIndices::BONEANGLE_ROTATE)*ci::app::getWindowHeight())+ci::app::getWindowHeight()/2));
            alpha.push_back(1.0f - sample->getData(MocapDeviceData::DataIndices::BONEANGLE_LATERAL));  //color it differently than the accel values
        }
    };
    
    //IDEA -- ratio between limbs? try wekinator, gtk -- closed/open, up/down -- can implement all these measures -- also in ITM -- verticality
    
};
    
    //needed for notch figure
    enum Axis {X=0, Y=1, Z=2};
    std::initializer_list<Axis> AxisList{Axis::X, Axis::Y, Axis::Z};
    
    //visualizes one bone -- NOTE:: this 2D class is now obsolete & doesn't work except as a parent class to the 3DBone
    //TODO: refactor out!!
    class MocapDataVisualizerNotchFigure2DBone : public MocapDataVisualizer
    {
    protected:
        std::string _name;
        MocapDataVisualizerNotchFigure2DBone *_parent;
        std::vector<ci::vec3> _angles;
        ci::vec3 _curAnchorPos; //this will be really vec2 for now... -- this is where the child bone connects to -- must keep track to maintin figure integrity
        ci::vec3 _boneLength;
        std::string _parentName;
        ci::vec3 lastAngle;
        ci::vec3 _origAnchorPos;
        ci::vec3 _curEndPoint;
        
    protected:
        Axis _axis;
    public:

        MocapDataVisualizerNotchFigure2DBone(std::string name, ci::vec3 startAnchorPos, ci::vec3 boneLength, std::string parentName, Axis axis, OutputSignalAnalysis *s1=NULL, MocapDataVisualizerNotchFigure2DBone *parent=NULL, int bufsize=48, SignalAnalysis *s2 = NULL)  : MocapDataVisualizer(s1, 0, bufsize, s2), lastAngle(0,0,0)
        {
            _name = name;
            _parent = parent;
            maxDraw = 1;
            _curAnchorPos = startAnchorPos;
            _origAnchorPos = ci::vec3(startAnchorPos.x, startAnchorPos.y, startAnchorPos.z);
            
            _boneLength = boneLength; //the x value is the thickness, the y value is the length, z also contributes to thickness in 3d, ignored in 2d test
            _parentName = parentName;
            
            _axis = axis;
        };
        
        void setParent(MocapDataVisualizerNotchFigure2DBone *parent)
        {
            _parent = parent;
        };
        
        std::string getParentName()
        {
            return _parentName;
        }
        
        ci::vec3 getCurEndPoint()
        {
            return _curEndPoint;
        }

        void setSignal(OutputSignalAnalysis *s1)
        {
            ugen=s1;
        };
        
        std::string getName()
        {
            return _name;
        }
        
        virtual void update(float seconds = 0)
        {
            if(ugen==NULL) return;
//            std::cout << "Bone " << _name << " is updating." << "Parent is: "<< getParentName() <<" and parent name: " << _parent->getName() << "and anchor pos: " << getAnchorPos() << "\n ";

            std::vector<MocapDeviceData *> buffer = ugen->getBuffer();
            if(buffer.size()<=0) return;
            
            points.clear();
            alpha.clear();
            _angles.clear();
            
            for(int i=buffer.size()-this->getNewSampleCount(); i<buffer.size(); i++)
            {
                MocapDeviceData *sample = buffer[i];
                _angles.push_back( sample->getRelativeBoneAngles() );
                lastAngle = sample->getRelativeBoneAngles();
            }
        };
        
        virtual double convertToRadiansAndRelativeNotchAngle(ci::vec3 angle, std::string name)
        {
            //idea -- for arms -- ADD the angles...??
            
            float a;
            if(_axis==Axis::X)
            {
                a = angle.x;
            }
            else if(_axis==Axis::Y)
            {
                a = angle.y;
            }
            else
            {
                a = angle.z;
            }

            a *= ( M_PI/180.0);
            
            if(!name.compare("LeftUpperArm") || !name.compare("RightUpperArm"))
            {
                a+= M_PI;
            }

            return a;
        }
        
        ci::vec3 getAxis()
        {
            if(_axis==Axis::X)
            {
                return ci::vec3(1.0f, 0.0f, 0.0f);
            }
            else if(_axis==Axis::Y)
            {
                return ci::vec3(1.0f, 0.0f, 0.0f);
            }
            else
            {
                return ci::vec3(1.0f, 0.0f, 0.0f);
            }
        }
        
        virtual ci::vec3 getBoneLength()
        {
            return _boneLength;
        }

        virtual ci::vec3 getAnchorPos()
        {
            return _curAnchorPos;
        }
        
        virtual ci::vec3 getRelativeAngle()
        {
            return lastAngle;
        }
        
        ci::vec3 getStartAnchorPos()
        {
            return _origAnchorPos;
        }
        
        ci::vec3 xform2DPointUsingCurModelView(ci::vec3 pt)
        {
            ci::mat4 modelView = ci::gl::getModelView();
            ci::vec4 row1(1, 0, 0, pt.x );
            ci::vec4 row2(0, 1, 0, pt.y );
            ci::vec4 row3(0, 0, 1, 0 );
            ci::vec4 row4(0, 0, 0, 1 );
            ci::mat4 anchor(row1, row2, row3, row4);
            ci::mat4 newAnchor = anchor * modelView;
            ci::vec4 mvm = newAnchor * ci::vec4(1,0,0,1);
            return ci::vec3(mvm[0], mvm[1], 0);
        }
        
        ci::vec3 xform3DPointUsingCurModelView(ci::vec3 pt)
        {
            ci::mat4 modelView = ci::gl::getModelView();
            ci::vec4 row1(1, 0, 0, pt.x );
            ci::vec4 row2(0, 1, 0, pt.y );
            ci::vec4 row3(0, 0, 1, pt.z );
            ci::vec4 row4(0, 0, 0, 1 );
            ci::mat4 anchor(row1, row2, row3, row4);
            ci::mat4 newAnchor = anchor * modelView;
            ci::vec4 mvm = newAnchor * ci::vec4(1,0,0,1);
            return ci::vec3(mvm[0], mvm[1], mvm[2]);
        }
    
        void doAnchorPosCalcs()
        {
            _curAnchorPos = xform3DPointUsingCurModelView(getStartAnchorPos());
            
            ci::gl::pushModelMatrix();
            ci::gl::translate(ci::vec3(0, -_boneLength.y, 0));
            _curEndPoint = xform3DPointUsingCurModelView(getStartAnchorPos());
            ci::gl::popModelMatrix();
        }
        
        float scaleValue(float val, float lastmin, float lastmax, float newmin, float newmax)
        {
            float newval = (val-lastmin)/(lastmax-lastmin); //-1to1 --- 0.5-(-1)/(2) = 1.5/2 = 0.75
            newval =  (newval*(newmax-newmin))+ newmin; //1to10 --- (0.75*9) + 1 = 0.75
            return newval;
        }
        
        ci::vec3 scaleTo3dCoords(ci::vec3 pt)
        {
//            ci::gl::drawSphere(ci::vec3(0, 12, 0), 0.1f);
//            ci::gl::drawSphere(ci::vec3(0, 0, 15), 0.1f);
//            ci::gl::drawSphere(ci::vec3(15, 0, 0), 0.1f);
//            gl::color(0,1,1,1);
//            ci::gl::drawSphere(ci::vec3(0, -15, 0), 0.1f);
//            ci::gl::drawSphere(ci::vec3(0, 0, -20), 0.1f);
//            ci::gl::drawSphere(ci::vec3(-20, 0, 0), 0.1f);
            
            //  ranges --> x = -20, 15; y = -15, 12; z=-20, 15
            
            float x = scaleValue(pt.x, 0,  ci::app::getWindowWidth(), -5, 10);
            float y = scaleValue(pt.y, 0, ci::app::getWindowHeight(), -5, 10);
            float z = scaleValue(pt.z, 0, ci::app::getWindowHeight(), -5, 10);//scaleValue(pt.z, ci::app::getWindowWidth(), ci::app::getWindowHeight(), -20, 15);
            
            return ci::vec3(x, y, z);

        }
        
        float scaleOneVal(float val, Axis axis)
        {
            if(_axis==Axis::X)
            {
                val = scaleValue(val, 0,  ci::app::getWindowWidth(), -5, 10);
            }
            else if(_axis==Axis::Y)
            {
                val = scaleValue(val, 0, ci::app::getWindowHeight(), -5, 10);
            }
            else
            {
                val = scaleValue(val, 0, ci::app::getWindowWidth(), -5, 10);
            }
            return val;
        }
        
        ci::vec2 parentTranslationsAndRotations()
        {
            ci::vec2 parentAnchor, curAnchor, whereAmI(0,0);
            if(_parent != NULL){
                whereAmI = _parent->parentTranslationsAndRotations();
                curAnchor.x = _parent->getStartAnchorPos().x - whereAmI.x;
                curAnchor.y = _parent->getStartAnchorPos().y - whereAmI.y;
                whereAmI.x+=curAnchor.x;
                whereAmI.y+=curAnchor.y;

//                ci::gl::translate(curAnchor);
//                ci::gl::rotate(convertToRadiansAndRelativeNotchAngle(lastAngle, getName()));
                
                ci::vec3 translateVector = scaleTo3dCoords(ci::vec3(curAnchor.x, curAnchor.y, 0));
                translateVector.z = 0;
//                translateVector.x = 0;

                ci::gl::translate(translateVector);

//                ci::gl::rotate(convertToRadiansAndRelativeNotchAngle(lastAngle, getName()));
                
                //this new rotate
                ci::gl::rotate (convertToRadiansAndRelativeNotchAngle(lastAngle, getName()), ci::vec3(0.0f, 1.0f, 0.0f));

//                ci::gl::rotate(ci::vec3(convertToRadiansAndRelativeNotchAngle(lastAngle, getName()), 0, 0));
            }
            return whereAmI;
        }
        
        virtual ci::vec3 translationsAndRotations3d()
        {
            ci::vec3 parentAnchor, curAnchor, whereAmI(0,0,0);
            if(_parent != NULL){
                whereAmI = _parent->translationsAndRotations3d();
                curAnchor.x = _parent->getStartAnchorPos().x - whereAmI.x;
                curAnchor.y = _parent->getStartAnchorPos().y - whereAmI.y;
                whereAmI.x+=curAnchor.x;
                whereAmI.y+=curAnchor.y;
                
                //                ci::gl::translate(curAnchor);
                //                ci::gl::rotate(convertToRadiansAndRelativeNotchAngle(lastAngle, getName()));
                
                ci::vec3 translateVector = scaleTo3dCoords(ci::vec3(curAnchor.x, curAnchor.y, 0));
                translateVector.z = 0;
                translateVector.x = 0;
                ci::gl::translate(translateVector);
                
                //                ci::gl::rotate(convertToRadiansAndRelativeNotchAngle(lastAngle, getName()));
                
                //this new rotate
                ci::gl::rotate (convertToRadiansAndRelativeNotchAngle(lastAngle, getName()), ci::vec3(1.0f, 0.0f, 0.0f));
                
                //                ci::gl::rotate(ci::vec3(convertToRadiansAndRelativeNotchAngle(lastAngle, getName()), 0, 0));
            }
            return whereAmI;
        }
        
        virtual void drawAnchorPoints()
        {
//            if(_name.compare("LeftForeArm") && _name.compare("RightForeArm"))
//                return;
            //draw the anchor point tho.
            ci::gl::color(1.0f, 1.0f, 1.0f, 1.0f);
//            ci::gl::drawSolidCircle(ci::vec2(getAnchorPos().x,getAnchorPos().y), 3);
            
            ci::vec3 endpt1 = scaleTo3dCoords(ci::vec3(getStartAnchorPos().x, getStartAnchorPos().y, 0 ));
            endpt1.z = 0;
            endpt1.x =0;
            
             ci::gl::drawSphere(endpt1, 0.1f);
             if(!_parent) return;
        
            
            ci::gl::color(1.0f, 0.0f, 1.0f, 0.4f);

            ci::vec3 endpt2 = scaleTo3dCoords(ci::vec3(_boneLength.x, _boneLength.y, 0));
            endpt2.z = 0;
            endpt2.x = 0;

            ci::gl::drawSphere(endpt2, 0.1f);
            
            ci::gl::color(1.0f, 1.0f, 1.0f, 1.0f);
        }
        
        virtual void draw(float seconds = 0)
        {
            if(!_parent) return;
            
            ci::gl::pushModelMatrix();
            ci::vec3 endpt1, endpt2;
           
            ci::gl::color(1.0f, 1.0f, 0.0f, 1.0f);
             
            ci::gl::rotate(M_PI, ci::vec3(1.0, 0.0, 0.0));

            ci::gl::translate(scaleOneVal(getStartAnchorPos().x, Axis::X)-5.0, 0, 0.0);
//            ci::gl::rotate (convertToRadiansAndRelativeNotchAngle(lastAngle, getName()), ci::vec3(1.0f, 0.0f, 0.0f));
//            
//            std::cout << "last angle: " << convertToRadiansAndRelativeNotchAngle(lastAngle, getName()) << std::endl;

            translationsAndRotations3d();
            endpt1 = ci::vec3(0, 0, 0);
            endpt2 = ci::vec3(0, scaleOneVal(-_boneLength.y, Axis::Y), 0);
            ci::gl::drawLine(endpt1, endpt2);
            ci::gl::popModelMatrix();
        
        }

    };
    
    //visualizes one 3D bone
    class MocapDataVisualizerNotchFigure3DBone : public MocapDataVisualizerNotchFigure2DBone
    {
    protected:
        ci::Color myColor;
        bool mDrawBoneDown;
    public:
        MocapDataVisualizerNotchFigure3DBone(std::string name, ci::vec3 startAnchorPos, ci::vec3 boneLength, std::string parentName, OutputSignalAnalysis *s1=NULL, MocapDataVisualizerNotchFigure3DBone *parent=NULL, int bufsize=48, SignalAnalysis *s2 = NULL) : MocapDataVisualizerNotchFigure2DBone(name, startAnchorPos, boneLength, parentName, Axis::X, s1, parent, bufsize, s2)
        {
            float r1 = float(std::rand())/float(RAND_MAX);
            float r2 = float(std::rand())/float(RAND_MAX);
            float r3 = float(std::rand())/float(RAND_MAX);
            myColor = ci::Color(r1, r2, r3);
            mDrawBoneDown = false;
            
        };
        
        void drawBoneDown(bool down = true)
        {
            mDrawBoneDown = down;
        }
        
        bool isDrawnDown()
        
        {
            return mDrawBoneDown;
        }
        
        virtual double radians(ci::vec3 angle, Axis axis)
        {
            //idea -- for arms -- ADD the angles...??
            
            float a;
            if(axis==Axis::X)
            {
                a = angle.x;
            }
            else if(axis==Axis::Y)
            {
                a = angle.y;
            }
            else
            {
                a = angle.z;
            }
            
            a *= ( M_PI/180.0);
        
            return a;
        }
        
        virtual ci::vec3 translationsAndRotations3d()
        {
            ci::vec3 parentAnchor, curAnchor, whereAmI(0,0,0);
            if(_parent != NULL){
                whereAmI = _parent->translationsAndRotations3d();
                curAnchor.x = _parent->getStartAnchorPos().x - whereAmI.x;
//                curAnchor.y = _parent->getStartAnchorPos().y - whereAmI.y;

                
                if(((MocapDataVisualizerNotchFigure3DBone *)_parent)->isDrawnDown())
                {
                    curAnchor.y = whereAmI.y-_parent->getStartAnchorPos().y ;
                }
                else
                {
                    curAnchor.y = _parent->getStartAnchorPos().y - whereAmI.y;
                }

                whereAmI.x+=curAnchor.x;
                if(((MocapDataVisualizerNotchFigure3DBone *)_parent)->isDrawnDown())
                {
                    whereAmI.y-=curAnchor.y;
                }
                else
                {
                    whereAmI.y+=curAnchor.y;
                }
                

                ci::vec3 translateVector = ci::vec3(curAnchor.x, curAnchor.y, 0);
                ci::gl::translate(translateVector);
//                std::cout << getName() << "-- translate: " << translateVector << "   curAnchor: " << curAnchor << std::endl;
                
                //this new rotate
                ci::gl::rotate (radians(lastAngle, Axis::X), ci::vec3(1.0f, 0.0f, 0.0f));
                ci::gl::rotate (radians(lastAngle, Axis::Y), ci::vec3(0.0f, 1.0f, 0.0f));
                ci::gl::rotate (radians(lastAngle, Axis::Z) , ci::vec3(0.0f, 0.0f, 1.0f));
            }
            return whereAmI;
        }
        
        ci::vec3 xform3DPtUsingCurTransformationMatrix(ci::vec3 pt)
        {
            ci::mat4 model = ci::gl::getModelMatrix();
            ci::vec4 input(pt.x, pt.y, pt.z, 1.0f);
            ci::vec4 output = model * input;
            return ci::vec3(output[0], output[1], output[2]);
        }
        
        virtual void drawAnchorPoints()
        {
            
            ci::gl::color(0.0f, 1.0f, 1.0f, 1.0f);
            ci::gl::drawSphere(_curAnchorPos, 0.1f);
            if(!_parent) return;
            
            ci::gl::color(1.0f, 0.0f, 1.0f, 1.0f);
            ci::gl::drawSphere(_curEndPoint, 0.1f);
        }
        
        virtual void doAnchorPosCalcs()
        {
            _curAnchorPos = xform3DPtUsingCurTransformationMatrix(ci::vec3(0, 0, 0));
            
            ci::gl::pushModelMatrix();
            
            if(mDrawBoneDown)
                ci::gl::translate(ci::vec3(0, _boneLength.y, 0));
            else
                ci::gl::translate(ci::vec3(0, -_boneLength.y, 0));
            
            _curEndPoint = xform3DPtUsingCurTransformationMatrix(ci::vec3(0, 0, 0));
            
            ci::gl::popModelMatrix();
        }
        
        virtual void draw(float seconds = 0)
        {
            if(!_parent) return;
            
            ci::vec3 endpt1, endpt2;
            endpt1 = ci::vec3(0, 0, 0);
            
            if(mDrawBoneDown)
                endpt2 = ci::vec3(0, _boneLength.y, 0);
            else endpt2 = ci::vec3(0, -_boneLength.y, 0);

            ci::gl::color(myColor);
            
            ci::gl::pushModelMatrix();
            ci::gl::rotate(M_PI, ci::vec3(1.0, 0.0, 0.0));
            translationsAndRotations3d();
            doAnchorPosCalcs();
            ci::gl::drawLine(endpt1, endpt2);
            ci::gl::popModelMatrix();
            
            drawAnchorPoints();
        }
    };
    
    //return a hard-coded bone visualizer given a name -- exact match
    class BoneFactory
    {
    protected:
        std::vector<std::string> names;
        std::vector<std::string> parents;
        double thickness;
        std::vector<double> length;
        std::vector<ci::vec3> anchorPos;
        std::vector<bool> drawDown;
    public:
        BoneFactory(double bodyStartX=0)
        {
            //hack hack
            names = {"Root", "Hip", "ChestBottom", "LeftUpperArm", "LeftForeArm", "RightUpperArm", "RightForeArm"};
            parents = {"", "Root", "Hip", "ChestBottom", "LeftUpperArm", "ChestBottom", "RightUpperArm"};
            drawDown = {false, false, false, true, true, true, true};
            
            thickness = 5;
            
//            double h = ci::app::getWindowHeight();
            double h = 20;
            double bodyStartY = 0; //start of hip
            
            //okay think about these a bit -- Note: height will be minused from anchor
            length = {0, h*0.15, h*0.35, h*0.1, h*0.1, h*0.1, h*0.1 };
            
            //ok, hand coded. let's see
            anchorPos.push_back(ci::vec3(bodyStartX, bodyStartY, 0.0)); //root
            anchorPos.push_back(ci::vec3(bodyStartX, bodyStartY-length[1], 0.0)); //hip
            anchorPos.push_back(ci::vec3(bodyStartX, anchorPos[1].y-(length[2]*0.75), 0.0)); //chest
            anchorPos.push_back(ci::vec3(bodyStartX, anchorPos[2].y-length[3], 0.0)); //left upper arm
            anchorPos.push_back(ci::vec3(bodyStartX, anchorPos[3].y-length[4], 0.0)); //left fore arm
            anchorPos.push_back(ci::vec3(bodyStartX, anchorPos[2].y-length[3], 0.0)); //right upper arm
            anchorPos.push_back(ci::vec3(bodyStartX, anchorPos[3].y-length[4], 0.0)); //right fore arm
            
        };
        
        //note: will need to set parent outside of this class, but this class can ID the parent.
        MocapDataVisualizerNotchFigure3DBone *createBone(std::string name_, int bufsize=48)
        {
            
            MocapDataVisualizerNotchFigure3DBone *bone = NULL;
            int index = getBoneIndex(name_);
            
            ci::vec3 len(0,0,0);
            if(index >= 0)
            {
                ci::vec3 len(0,0,0);
                len.x = thickness;
                len.z = thickness;
                len.y = length[index];
                
                bone = new MocapDataVisualizerNotchFigure3DBone(name_, anchorPos[index], len, parents[index]);
                bone->drawBoneDown(drawDown[index]);
            }
            return bone;
        };

        std::string getName(int index)
        {
            return names[index];
        }
        
        int getBoneCount()
        {
            return names.size();
        }
        
        int getBoneIndex(std::string name_)
        {
            auto iter = std::find(names.begin(), names.end(), name_);
            int index = iter - names.begin();
            return index;
        };
        
        
    };
    
    //creates all bones that I am currently using
    //TODO: Draw a static shoulder bone......
    //See if that fixes some disrepencies...
    class NotchBoneFigure : public MocapDataVisualizer
    {
    protected:
        BoneFactory factory;
        std::vector<MocapDataVisualizerNotchFigure3DBone *> bones;
        
        void setParents()
        {
            for(int i=0; i<bones.size(); i++)
            {
                std::string parentName = bones[i]->getParentName();
                MocapDataVisualizerNotchFigure2DBone *p=getBone(parentName);
//                if(p==NULL)
//                    std::cout << bones[i] << " has a null parent\n";
                bones[i]->setParent(p);
            }
        };
    public:
        NotchBoneFigure(double bodyStartX=0) : MocapDataVisualizer(NULL, 0, 48, NULL), factory(bodyStartX)
        {
            //create all the bones
            for(int i=0; i<factory.getBoneCount(); i++)
            {
                bones.push_back(factory.createBone(factory.getName(i)));
            }
            setParents();
        };
        
        MocapDataVisualizerNotchFigure3DBone *getBone(std::string name_)
        {
            int index = factory.getBoneIndex(name_);
            return getBone(index);
        };
        
        MocapDataVisualizerNotchFigure3DBone *getBone(int index)
        {
            if(index > -1 && index < bones.size())
                return bones[index];
            else return NULL;
        };
        
        int getBoneCount()
        {
            return factory.getBoneCount();
        }

        void setInputSignal(std::string name, OutputSignalAnalysis *s1)
        {
            int index = factory.getBoneIndex(name);
            if(index > -1)
            {
                bones[index]->setSignal(s1);
//                std::cout << "Input signal set for Bone " << name << " is set.\n ";
            }
            else std::cout << "NotchBoneFigure Error: Bone " << name << " is not found\n. ";

        }
        
        virtual void update(float seconds = 0)
        {
            for(int i=0; i<bones.size(); i++)
            {
                bones[i]->update();
            }
        };
        
        virtual void draw()
        {
            for(int i=0; i<bones.size(); i++)
            {
                bones[i]->draw();
            }
        }
        
        
        
    };
    
    class FigureMeasure : public SignalAnalysis
    {
    protected:
        NotchBoneFigure *figure;
        
        //scales from 0 to 1
        float scaledValue0to1(float input, float minVal, float maxVal)
        {
            float output = (input-minVal)/(maxVal-minVal);
            
            //cap values 0 to 1
            output = std::min(output, 1.0f);
            output = std::max(output, 0.0f);
            
            return output;
        }
    public:
        FigureMeasure(NotchBoneFigure *figure_, int bufnum=48) : SignalAnalysis(NULL, bufnum, NULL)
        {
            figure = figure_;
        }
        
        virtual void draw()
        {
            
        }
    };
    
    //TODO
    class ConvexShapeFromBoneEndPoints : public FigureMeasure
    {
        
    };
    
    //find center of mass -- stable / unstable? -- wekinator?
    class Centroid : public FigureMeasure
    {
        //http://adaptivemap.ma.psu.edu/websites/moment_intergrals/centroids_3D/centroids3D.html
    };
    
    //Doesn't really work for bending yet...
    //idea -- create a cone instead of a cylinder that gets larger or smaller... ?! or a frustrum?
    class ContractionIndex : public FigureMeasure
    {
    protected:
        float mTotal, mScaledTotal;
        float mRadius, mHeight, mVolume;
        //        float mSumDistanceFromRoot; //sum of the distance of all end points to the root bone //note:looks useless as a measure
    public:
        ContractionIndex(NotchBoneFigure *figure_, int bufnum=48) : FigureMeasure(figure_, bufnum)
        {
        }
        
        virtual void update(float seconds = 0)
        {

            //create a cylinder using furthest points up/down & left/right
        
            //lowest point is the root, so hip anchor pos
            ci::vec3 p1 = figure->getBone("Root")->getAnchorPos();
            
            //find highest point (Y value)
            ci::vec3 p2 = findHighestPoint();
            
            //find furthest point left from chest anchor
            ci::vec3 leftP = findMostLeftPoint();
            
            //find furthest point from this left pt -- lets say its right altho could be z direction
            ci::vec3 rightP = findFarestPointFrom(leftP);

            //find the distance btw left & right
             mRadius = ci::distance(ci::vec2(leftP.x,leftP.z) , ci::vec2(rightP.x,rightP.z));
            
            //find y distance between p1  & p2
             mHeight = p2.y - p1.y;
            
            //ok, find the volume of this cylinder, which will be the contraction index
            //a thought (for tomorrow) -- currently, the chest is super long & always straight -- which is skewing the results -- could make it shorter
            //or else remove torso movement from this equation for now.
             mVolume = M_PI * mRadius * mRadius * mHeight;
             mVolume = scaleVolume();
            
//            mSumDistanceFromRoot = getSummedDistanceFromRoot();
//            mSumDistanceFromRoot = scaleSummedDistance();
            
            //let's look at the values
//            std::cout << "update: volume: " << mVolume << "  height:" << mHeight << " radius:" << mRadius <<  " SumDistanceFromRoot:" << mSumDistanceFromRoot << std::endl;
        }
        
        //hack hack hack -- quick and dirty - bypassing my motiondata class... yikes
        float scaleVolume()
        {
            const float MAX_RECORDED_VOLUME_EST = 650;
            const float MIN_RECORDED_VOLUME_EST = 10;
            return scaledValue0to1(mVolume,MIN_RECORDED_VOLUME_EST, MAX_RECORDED_VOLUME_EST );
        }
        
        //hack hack hack -- quick and dirty - bypassing my motiondata class... yikes
//        float scaleSummedDistance()
//        {
//            const float MAX_RECORDED_DIST_EST = 40;
//            const float MIN_RECORDED_DIST_EST = 34;
//
//            //scale
//            float dist = (mSumDistanceFromRoot-MIN_RECORDED_DIST_EST) / (MAX_RECORDED_DIST_EST-MIN_RECORDED_DIST_EST);
//
//            //cap values 0 to 1
//            dist = std::min(dist, 1.0f);
//            dist = std::max(dist, 0.0f);
//
//            return dist;
//        }
        
//        float getSummedDistanceFromRoot()
//        {
//            ci::vec3 root = figure->getBone(0)->getCurEndPoint(); //root bone
//            float sum = 0;
//
//            for(int i=1; i<figure->getBoneCount(); i++)
//            {
//                sum += ci::distance(root, figure->getBone(i)->getCurEndPoint());
//            }
//            return sum;
//        }
        
        virtual void draw()
        {
            ci::gl::color(1.0f, 1.0f, 1.0f, 1.0f );

            drawCylinder(mRadius, mHeight);
        }
        
        
        virtual void drawCylinder(float radius, float height)
        {
            auto lambert = ci::gl::ShaderDef().lambert().color();
            ci::gl::GlslProgRef shader = ci::gl::getStockShader( lambert );
            ci::gl::BatchRef cylinder = ci::gl::Batch::create( ci::geom::Cylinder(), shader );
            
            ci::gl::pushModelMatrix();
            ci::gl::scale( ci::vec3( radius, height/2.0f, radius ) );
            ci::gl::color(  0, 1, 0, 0.25f);
            cylinder->draw();
            ci::gl::popModelMatrix();
        }
        
        ci::vec3 findHighestPoint()
        {
            
            ci::vec3 pt = figure->getBone(0)->getCurEndPoint();

            for(int i=1; i<figure->getBoneCount(); i++)
            {
                if(pt.y < figure->getBone(i)->getCurEndPoint().y)
                    pt = figure->getBone(i)->getCurEndPoint();
            }
            
            return pt;
        }
        
        ci::vec3 findMostLeftPoint()
        {
            //NOTE: I know that Root is the 1st bone, so I'm skipping
            //Also, this will be the point that is furthest away in the x-axis -- may add z later?
            
            ci::vec3 startingPoint3d = figure->getBone("Hip")->getAnchorPos();
            ci::vec2 startingPt(startingPoint3d.x, startingPoint3d.z);
            
            ci::vec3 pt = figure->getBone(1)->getCurEndPoint();
            
            for(int i=2; i<figure->getBoneCount(); i++)
            {
                ci::vec3 endingPoint3 = figure->getBone(i)->getCurEndPoint();
                ci::vec2 endingPt(endingPoint3.x, endingPoint3.z);
                ci::vec2 pt2(pt.x, pt.z);
                
                //make sure it is on the left side (x)
                if(startingPt.x - endingPt.x >= 0){
                    if(ci::distance(pt2, startingPt) >  ci::distance(endingPt, startingPt))
                        pt = endingPoint3;
                }
            }

            return pt;
        }
        
        ci::vec3 findFarestPointFrom(ci::vec3 farestPt)
        {
            //NOTE: I know that Root is the 1st bone, so I'm skipping
            //Also, this will be the point that is furthest away in the x-axis -- may add z later?
            
            
            ci::vec2 fPt2(farestPt.x, farestPt.z);
            
            ci::vec3 pt = figure->getBone(1)->getCurEndPoint();
            
            for(int i=2; i<figure->getBoneCount(); i++)
            {
                ci::vec3 endingPoint3 = figure->getBone(i)->getCurEndPoint();
                ci::vec2 endingPt(endingPoint3.x, endingPoint3.z);
                ci::vec2 pt2(pt.x, pt.z);
                
                if(ci::distance(pt2, fPt2) <  ci::distance(endingPt, fPt2))
                    pt = endingPoint3;
            }
            
            return pt;
        }
        
        float getDistance(std::string boneName1, std::string boneName2, bool useEnd=true, bool useEnd2=true)
        {
            MocapDataVisualizerNotchFigure3DBone *bone1 = figure->getBone(boneName1);
            MocapDataVisualizerNotchFigure3DBone *bone2 = figure->getBone(boneName2);
            
            ci::vec3 pt1, pt2;
            
            if(useEnd)
                pt1 =  bone1->getCurEndPoint();
            else pt1 = bone1->getAnchorPos();
            
            if(useEnd2)
                pt2 =  bone2->getCurEndPoint();
            else pt2 = bone2->getAnchorPos();
            
            return ci::distance(pt1, pt2);
        }
        
        std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            
            ci::osc::Message msg;
            msg.setAddress(CI_OSCMESSAGE);
            msg.append(mVolume);
            
            msgs.push_back(msg);
            
            return msgs;
        }
        
    };
    
    class ArmHeight : public FigureMeasure
    {
    protected:
        float mArmHeight, mLeftArmHeight, mRightArmHeight;
    public:
        ArmHeight(NotchBoneFigure *figure_, int bufnum=48) : FigureMeasure(figure_, bufnum)
        {
        }
        
        //TODO: calibrate these values -- also the CI
        virtual void update(float seconds = 0)
        {
            mLeftArmHeight= armDistanceFromHipinY("Left");
            const float MAX_RECORDED_LEFTARM_HEIGHT_EST = 17.8993;
            const float MIN_RECORDED_LEFTARM_HEIGHT_EST = 13.842;
            
            mRightArmHeight= armDistanceFromHipinY("Right");
            const float MAX_RECORDED_RIGHTARM_HEIGHT_EST = 16.1399;
            const float MIN_RECORDED_RIGHTARM_HEIGHT_EST = 13.8946;
            
            const float MAX_RECORDED_ARM_HEIGHT_EST = 32.3537;
            const float MIN_RECORDED_ARM_HEIGHT_EST = 27.7449;
            mArmHeight = mLeftArmHeight + mRightArmHeight;
            
            //make sure left & right scaling is done AFTER scalling the combined arm heights since that is based on the unscaled sum
            mArmHeight = scaledValue0to1(mArmHeight, MIN_RECORDED_ARM_HEIGHT_EST, MAX_RECORDED_ARM_HEIGHT_EST);
            mRightArmHeight = scaledValue0to1(mRightArmHeight, MIN_RECORDED_RIGHTARM_HEIGHT_EST, MAX_RECORDED_RIGHTARM_HEIGHT_EST);
            mLeftArmHeight = scaledValue0to1(mLeftArmHeight, MIN_RECORDED_LEFTARM_HEIGHT_EST, MAX_RECORDED_LEFTARM_HEIGHT_EST);
            
            std::cout << "mArmHeight:" << mArmHeight << " mLeftArmHeight: " << mLeftArmHeight << " mRightArmHeight:" << mRightArmHeight << std::endl;
        }
        
        float armDistanceFromHipinY(std::string whichArm)
        {
            ci::vec3 pt = figure->getBone("Hip")->getAnchorPos();
            ci::vec3 pt2 =  figure->getBone(whichArm+"UpperArm")->getAnchorPos();
            ci::vec3 pt3 =  figure->getBone(whichArm+"ForeArm")->getAnchorPos();
            
            return (pt2.y-pt.y) + (pt3.y-pt.y);

        }
        
        std::vector<ci::osc::Message> getOSC()
        {
            std::vector<ci::osc::Message> msgs;
            
            ci::osc::Message msg;
            msg.setAddress(ARMHEIGHT_OSCMESSAGE);
            msg.append(mArmHeight);
            msg.append(mLeftArmHeight);
            msg.append(mRightArmHeight);

            msgs.push_back(msg);
            
            return msgs;
        }
        
    };


};

#endif /* UGENs_h */
