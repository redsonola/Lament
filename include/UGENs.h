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
    
    //visualizes one bone
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
            
            
            //update anchor pos...
//            lastAngle = _angles[_angles.size()-1]; //draw only last position
//            doAnchorPosCalcs();
//            double rotateAngle = convertToRadiansAndRelativeNotchAngle(lastAngle, getName()) ; //convert to radians
//
//            double radius = _boneLength.y;
//            //hack hack hack
//            if(!_name.compare("ChestBottom"))radius = _boneLength.y*0.65;
//
//            _curAnchorPos.x = radius*cos(-rotateAngle  + M_PI_2) + _parent->getAnchorPos().x;
//            _curAnchorPos.y = radius*-sin(-rotateAngle + M_PI_2) + _parent->getAnchorPos().y;
            
//            _curAnchorPos.x = radius*cos(-rotateAngle ) + _parent->getAnchorPos().x;
//            _curAnchorPos.y = radius*-sin(-rotateAngle ) + _parent->getAnchorPos().y;


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
//            if(!name.compare("LeftForeArm") || !name.compare("RightForeArm"))
//            {
//                a+= M_PI;
////                std::cout << name << ": " << _axis << " angle " << a/M_PI << std::endl;
////                a=-a;
//            }
            return a;
        }
        
        virtual ci::vec3 getBoneLength()
        {
            return _boneLength;
        }
        
        virtual double getParentNotchAngle()
        {
            float parentAngle;
            if(_axis==Axis::X)
            {
                parentAngle = _parent->getRelativeAngle().x;
            }
            else if(_axis==Axis::Y)
            {
                parentAngle = _parent->getRelativeAngle().y;
                
            }
            else
            {
                parentAngle = _parent->getRelativeAngle().z;
            }
            
//            if(!_name.compare("LeftUpperArm") || !_name.compare("RightUpperArm"))
//            {
//                parentAngle+= M_PI;
//            }
//            if(!_name.compare("LeftForeArm") || !_name.compare("RightForeArm"))
//            {
//                parentAngle+= M_PI;
//            }
            return parentAngle;
            
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
    
        
        //ok now just do the calculations...
        ci::vec2 parentTranslationsAndRotationsCalcs()
        {
            //ok need to now do the calcs
            //            _curAnchorPos.x = radius*cos(-rotateAngle  + M_PI_2) + _parent->getAnchorPos().x;
            //            _curAnchorPos.y = radius*-sin(-rotateAngle + M_PI_2) + _parent->getAnchorPos().y;
            
            ci::vec2 parentAnchor, curAnchor, whereAmI(0,0);
            if(_parent != NULL){
                whereAmI = _parent->parentTranslationsAndRotationsCalcs();
                curAnchor.x = _parent->getStartAnchorPos().x - whereAmI.x;
                curAnchor.y = _parent->getStartAnchorPos().y - whereAmI.y;
                whereAmI.x+=curAnchor.x;
                whereAmI.y+=curAnchor.y;
                
                float rotateAngle = convertToRadiansAndRelativeNotchAngle(lastAngle, getName());
//
//                x’ = x cos β – y sin β
//
//                y’ = x sin β + y cos β
                
//                whereAmI.x -= _parent->getStartAnchorPos().x;
//                whereAmI.y -= _parent->getStartAnchorPos().y;

                whereAmI.x += _boneLength.x*cos(-(rotateAngle + M_PI_2));
                whereAmI.y += _boneLength.y*-sin(-(rotateAngle + M_PI_2));
                
//                ci::gl::rotate(convertToRadiansAndRelativeNotchAngle(lastAngle, getName()));
                
                //ci::gl::drawSolidRect(ci::Rectf(ci::vec2(0,0), ci::vec2(_boneLength.x,-_boneLength.y))); //y is minus bc it is drawn upward from the anchor position
                
                //can you get the transformation matrix from OpenGL????

            }
            //            std::cout <<_name << ": Where am I? " << whereAmI.x << "," << whereAmI.y << std::endl;
            return whereAmI;
        }
        
        void doAnchorPosCalcs()
        {
            ci::vec2 curAnchor = ci::vec2(_curAnchorPos.x, _curAnchorPos.y);
            
//            mat4 getModelMatrix ()
//            mat4 getViewMatrix ()
//            mat4 getProjectionMatrix ()
//            mat4 getModelView()

//            Matrix4f::transformPoint ->  vec4 pt = mat4() * vec4(1,0,0,1)
            
            ci::mat4 modelView = ci::gl::getModelView();
            ci::mat4 proj = ci::gl::getProjectionMatrix();


            ci::vec4 row1(1, 0, 0, getStartAnchorPos().x );
            ci::vec4 row2(0, 1, 0, getStartAnchorPos().y );
            ci::vec4 row3(0, 0, 1, 0 );
            ci::vec4 row4(0, 0, 0, 1 );
            ci::mat4 anchor(row1, row2, row3, row4);
            ci::mat4 newAnchor = anchor * modelView;
            
            ci::vec4 mvm = newAnchor * ci::vec4(1,0,0,1);
            std::cout << mvm << std::endl;
//            std::cout << proj << std::endl;

            _curAnchorPos = ci::vec3(newAnchor[0][3], newAnchor[1][3], 0);
//

            
//            ci::vec3 pointInObjectSpace = ci::vec2(curAnchor.x, curAnchor.y, 0);
//            ci::vec2 pov = modelView.  transformPoint( pointInObjectSpace );
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

                ci::gl::translate(curAnchor);
                ci::gl::rotate(convertToRadiansAndRelativeNotchAngle(lastAngle, getName()));

                //ci::gl::drawSolidRect(ci::Rectf(ci::vec2(0,0), ci::vec2(_boneLength.x,-_boneLength.y))); //y is minus bc it is drawn upward from the anchor position



            }
//            std::cout <<_name << ": Where am I? " << whereAmI.x << "," << whereAmI.y << std::endl;
            return whereAmI;
        }
        
        void drawAnchorPoints()
        {
            //draw the anchor point tho.
            ci::gl::color(1.0f, 1.0f, 1.0f, 1.0f);
            ci::gl::drawSolidCircle(ci::vec2(getAnchorPos().x,getAnchorPos().y), 3);
            if(!_parent) return;
            
            ci::gl::color(1.0f, 0.0f, 1.0f, 0.4f);
            ci::gl::drawSolidCircle(ci::vec2(_parent->getAnchorPos().x,_parent->getAnchorPos().y), 3);
            
            ci::gl::color(1.0f, 1.0f, 1.0f, 1.0f);
        }
        
        virtual void draw(float seconds = 0)
        {

            if(!_parent) return;

            ci::gl::color(1.0f, 0.5f, 0.5f, 0.5f);
            float rotateAngle = convertToRadiansAndRelativeNotchAngle(lastAngle, getName()); //convert to radians
            
//float rotateAngle = 0; //see the figure w/no rotation
            
            ci::gl::pushModelMatrix();
//            ci::gl::translate(_parent->getAnchorPos());
//            std::cout << "------\n";
            ci::vec2 whereAmI = parentTranslationsAndRotations();
            doAnchorPosCalcs();
//            ci::gl::rotate(rotateAngle);

//            ci::gl::translate(whereAmI);
//            std::cout << "Final where Am I? "<< whereAmI << std::endl;
//            std::cout << "Vs Anchor Pos :" << _parent->getAnchorPos() << std::endl;

//            std::cout << "------\n";
            ci::gl::drawSolidRect(ci::Rectf(ci::vec2(0,0), ci::vec2(_boneLength.x,-_boneLength.y))); //y is minus bc it is drawn upward from the anchor position
            ci::gl::popModelMatrix();

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
    public:
        BoneFactory(double bodyStartX=ci::app::getWindowWidth() * 0.5)
        {
            //hack hack
            names = {"Root", "Hip", "ChestBottom", "LeftUpperArm", "LeftForeArm", "RightUpperArm", "RightForeArm"};
            parents = {"", "Root", "Hip", "ChestBottom", "LeftUpperArm", "ChestBottom", "RightUpperArm"};
            
            thickness = 5;
            
            double h = ci::app::getWindowHeight();
            double bodyStartY = ci::app::getWindowHeight() * 0.85; //start of hip
            
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
            
//            //get anchor points up to left upper fore arm
//            for(int i=1; i<length.size()-2; i++)
//            {
//                anchorPos.push_back(ci::vec3(bodyStartX, anchorPos[i-1].y-length[i], 0.0f));
//            }
//
//            //right arm is in same pos as left arm
//            anchorPos.push_back(anchorPos[3]);
//            anchorPos.push_back(anchorPos[4]);
            
        };
        
        //note: will need to set parent outside of this class, but this class can ID the parent.
        MocapDataVisualizerNotchFigure2DBone *createBone(std::string name_, Axis axis, int bufsize=48)
        {
            
            MocapDataVisualizerNotchFigure2DBone *bone = NULL;
            int index = getBoneIndex(name_);
            
            ci::vec3 len(0,0,0);
            if(index >= 0)
            {
                ci::vec3 len(0,0,0);
                len.x = thickness;
                len.z = thickness;
                len.y = length[index];
                
                bone = new MocapDataVisualizerNotchFigure2DBone(name_, anchorPos[index], len, parents[index], axis);
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
    class NotchBoneFigureVisualizer : public MocapDataVisualizer
    {
    protected:
        BoneFactory factory;
        std::vector<MocapDataVisualizerNotchFigure2DBone *> bones;
        
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
        NotchBoneFigureVisualizer(Axis axis, double bodyStartX=ci::app::getWindowWidth() * 0.5) : MocapDataVisualizer(NULL, 0, 48, NULL), factory(bodyStartX)
        {
            //create all the bones
            for(int i=0; i<factory.getBoneCount(); i++)
            {
                bones.push_back(factory.createBone(factory.getName(i), axis));
            }
            setParents();
        };
        
        MocapDataVisualizerNotchFigure2DBone *getBone(std::string name_)
        {
            int index = factory.getBoneIndex(name_);
//            std::cout << name_ << " has the index: " << index << "\n";

            if(index > -1 && index < bones.size())
                return bones[index];
            else return NULL;
        };
        
        void setInputSignal(std::string name, OutputSignalAnalysis *s1)
        {
            int index = factory.getBoneIndex(name);
            if(index > -1)
            {
                bones[index]->setSignal(s1);
//                std::cout << "Input signal set for Bone " << name << " is set.\n ";
            }
            else std::cout << "NotchBoneFigureVisualizer Error: Bone " << name << " is not found\n. ";

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

    
//    class MocapDataVisualizerNotchBoneFigure2D : public MocapDataVisualizer
//    {
//    public:
//        MocapDataVisualizerNotchBoneFigure2D(OutputSignalAnalysis *s1, int _maxDraw=25,int bufsize=48, SignalAnalysis *s2 = NULL)  : MocapDataVisualizer(s1, _maxDraw, bufsize, s2)
//        {
//
//
//        };
//
//        virtual void update(float seconds = 0)
//        {
//
////            std::vector<MocapDeviceData *> buffer = ugen->getBuffer();
////            if(buffer.size()<maxDraw) return; //ah well I don't want to handle smaller buffer sizes for this function. feel free to implement that.
////
////            points.clear();
////            alpha.clear();
////            for(int i=buffer.size()-this->getNewSampleCount(); i<buffer.size(); i++)
////            {
////                MocapDeviceData *sample = buffer[i];
////
////            }
//        };
//
//        virtual void draw()
//        {
//            ci::gl::color(1.0f, 0.5f, 0.5f, 0.5f);
//            drawFigure();
//        }
//
//
//        void drawFigure()
//        {
//            //TODO: add Z
//            //TODO:figure prob is just all one line @ 0 degrees, so that should be the start
//
//            //chest and hip
//            float chestLength = ci::app::getWindowHeight() * 0.5;
//            float lineWidth=5.0f;
//            float bodyStartY = ci::app::getWindowHeight() * 0.1;
//            float bodyStartX = ci::app::getWindowWidth() * 0.5;
//            float hipLength = ci::app::getWindowHeight() * 0.15;
//
//            //arm values
//            float upperArmStartY = bodyStartY + ci::app::getWindowHeight() * 0.25;
//            float upperArmLeftEndX = bodyStartX + ci::app::getWindowWidth() * 0.15;
//            float upperArmRightEndX = bodyStartX - ci::app::getWindowWidth() * 0.15;
//
//            float foreArmLeftEndX = upperArmLeftEndX + ci::app::getWindowWidth() * 0.15;
//            float foreArmRightEndX = upperArmRightEndX - ci::app::getWindowWidth() * 0.15;
//
//
//            ci::vec2 chestStart(bodyStartX, bodyStartY);
//            ci::vec2 chestEnd(bodyStartX+lineWidth, bodyStartY+chestLength);
//            ci::vec2 hipEnd(bodyStartX, bodyStartY+chestLength+hipLength );
////            std::cout << "drawing... "  << chestStart << "," << chestEnd << "\n";
//
//            ci::vec2 upperArmStart(bodyStartX, upperArmStartY);
//            ci::vec2 leftArmEnd(bodyStartX+lineWidth, upperArmStartY+ci::app::getWindowWidth() * 0.15);
//            ci::vec2 rightArmEnd(bodyStartX+lineWidth, upperArmStartY);
//
//            ci::vec2 leftFormArmEnd(bodyStartX, upperArmStartY);
//            ci::vec2 rightForeArmEnd(bodyStartX, upperArmStartY);
//
//
//            //NOTE -- only one per... need to make mo' complicated :(
//            ci::gl::drawSolidRect(ci::Rectf(chestStart, chestEnd)); //chest
//
//            ci::gl::drawSolidRect(ci::Rectf(chestEnd, hipEnd)); //hip
//
//            std::vector<MocapDeviceData *> buffer = ugen->getBuffer();
//            if (buffer.size() <= 0 )
//                return;
//            //next step -- if this is the upper arm, draw... will need to refactor so that it knows its parent bone tho... yikes. also this
//            //ugen needs to know its parent. Good to know.
//            MocapDeviceData *lastSample = buffer[buffer.size()-1];
//            float upperArmRotate = lastSample->getData(MocapDeviceData::DataIndices::RELATIVE_TILT) * (M_PI/180.0f);
//
//            ci::gl::pushModelMatrix();
//
//            ci::gl::translate(upperArmStart);
//            ci::gl::rotate(upperArmRotate);
//            ci::gl::drawSolidRect(ci::Rectf(ci::vec2(0,0), ci::vec2(5,ci::app::getWindowWidth() * 0.15))); //left upper arm
//            ci::gl::popModelMatrix();
//
//
////            ci::gl::drawSolidRect(ci::Rectf(upperArmStart, rightArmEnd)); //right upper arm
////
////            ci::gl::drawSolidRect(ci::Rectf(leftArmEnd, leftFormArmEnd)); //left upper arm
////
////            ci::gl::drawSolidRect(ci::Rectf(rightArmEnd, rightForeArmEnd)); //right upper arm
//
//
////            gl::pushModelMatrix();
////            gl::translate( ... );
////            gl::rotate( ... );
////            gl::draw( ... );
////            gl::popModelMatrix();
//        }
//    };




};

#endif /* UGENs_h */
