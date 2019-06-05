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
                
                for(int i= MocapDeviceData::DataIndices::BONEANGLE_TILT; i<=MocapDeviceData::DataIndices::BONEANGLE_TILT+2; i++)
                {
                    mdd->setData(i, mocapDeviceAvg(data1, start, end, i));
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
//                    std::cout << std::endl;
                    
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
                
                    //derivative of bone angles
                    for(int j= MocapDeviceData::DataIndices::BONEANGLE_TILT; j<=MocapDeviceData::DataIndices::BONEANGLE_LATERAL; j++)
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
        void draw()
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


};

#endif /* UGENs_h */
