//
//  MotionCaptureData.h
//  VideoAndOSCLab
//
//  Created by courtney on 10/22/18.
//

#ifndef MotionCaptureData_h
#define MotionCaptureData_h

#define WIIMOTE_ACCELMAX 1.0f
#define WIIMOTE_ACCELMIN 0.0f
#define IPHONE_ACCELMAX 50.0f
#define IPHONE_ACCELMIN -50.0f

//dunno yet need to check documentation
#define NOTCH_ACCELMAX 1.0f
#define NOTCH_ACCELMIN -1.0f

//for some reason, this will cause unexpected errors.
#define DEVICE_ARG_COUNT_MAX 24

namespace CRCPMotionAnalysis {


class MocapDeviceData
{
    
    protected:
        double data[DEVICE_ARG_COUNT_MAX+1];
        float quaternion[4];
        float orientationMatrix[3];
        bool isAccelScaled;
    
        //the default is wiimote
        virtual double getAccelMax(){ return WIIMOTE_ACCELMAX; };
        virtual double getAccelMin(){ return WIIMOTE_ACCELMIN; };
    public:
                                                                                                                //see documentation of these angles in the main
        enum DataIndices { INDEX=0, TIME_STAMP=1, ACCELX=2, ACCELY=3, ACCELZ=4, GYROX=11, GYROY=12, GYROZ=13, BONEANGLE_TILT=14, BONEANGLE_ROTATE=15, BONEANGLE_LATERAL=16, RELATIVE_TILT=17, RELATIVE_ROTATE=18, RELATIVE_LATERAL=19, ANGVEL_TILT=20, ANGVEL_ROTATE=21, ANGVEL_LATERAL=22, QX=23, QY=24, QZ=25, QA=26 };
        enum MocapDevice { WIIMOTE=0, IPHONE=1, NOTCH=2 };
    
        MocapDevice getDeviceType()
        {
            return device;
        };
    
        //scale accel to 0 - 1 -- obv. not needed if wiimote
        //assumes all axes are on same scale
        void scaleAccel()
        {
            if(isAccelScaled) return;
            
            for (int i = ACCELX; i <= ACCELZ; i++)
            {
                //take it to 0..1
                data[i] =  ( data[i] - getAccelMin() )  / ( getAccelMax() - getAccelMin() );
            }
            
            //hack hack - TODO: refactor
            for (int i = BONEANGLE_TILT; i <= BONEANGLE_ROTATE; i++)
            {
                //take it to 0..1
                data[i] =  ( data[i] - getAccelMin() )  / ( getAccelMax() - getAccelMin() );
            }
            
            isAccelScaled = true;
        };
    
        std::string toString()
        {
            std::stringstream sstr;
            
            for(int i=0; i<DEVICE_ARG_COUNT_MAX+1; i++)
            {
                if(data[i]!=NO_DATA)
                {
                    sstr << data[i] << ",";
                }
                
            }
            return sstr.str();
        }
    
        //return sensor data as a string
        std::string str()
        {
            std::stringstream ss;
            
            for (int i = 0; i < DEVICE_ARG_COUNT_MAX; i++)
            {
                ss << data[i];
                if (i < DEVICE_ARG_COUNT_MAX - 1) ss << ",";
            }
            ss << std::endl;
            
            return ss.str();
        };
        
        inline double getTimeStamp(){ return data[1]; };
        inline ci::vec3 getAccelData()
        {
            ci::vec3 accelData(data[ACCELX], data[ACCELY], data[ACCELZ]);
            return accelData;
        };
        inline void setAccelData(ci::vec3 d)
        {
            data[ACCELX] = d.x;
            data[ACCELY] = d.y;
            data[ACCELZ] = d.z;
        };
        inline ci::vec3 getGyro()
        {
            ci::vec3 gyro(data[GYROX], data[GYROY], data[GYROZ]);
            return gyro;
        };
        inline void setGyro(ci::vec3 d)
        {
            data[GYROX] = d.x;
            data[GYROY] = d.y;
            data[GYROZ] = d.z;
        };

        inline void setData(int index, double d)
        {
            data[index] = d;
        }
        inline double getData(int index)
        {
            if(index < 20)
            {
                return data[index];
            }
            else if (index < 24)
            {
                int i = index - 20;
                return getQuarternion(i);
            }
            else
            {
                std::cout << "Warning! Motion Sensor Data: Out of Range! Index: " << index << "\n";
                return NO_DATA;
            }
            
        }
        inline float getQuarternion(int index)
        {
            assert( index < 4 && index >= 0 );
            return quaternion[index];
        };
        
        inline ci::vec4 getQuarternionVec4d()
        {
            return ci::vec4(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        };
        
        inline void setQuarternion(ci::vec4 quat)
        {
            quaternion[0]=quat[0];
            quaternion[1]=quat[1];
            quaternion[2]=quat[2];
            quaternion[3]=quat[3];
        };
        
        inline void setQuarternion(float x, float y, float z, float angle)
        {
            quaternion[0] = x;
            quaternion[1] = y;
            quaternion[2] = z;
            quaternion[3] = angle;
            
        };
    
    inline ci::vec3 getBoneAngles()
    {
        return ci::vec3(getData(BONEANGLE_TILT), getData(BONEANGLE_ROTATE), getData(BONEANGLE_LATERAL));
    };
    
        inline ci::vec3 getRelativeBoneAngles()
        {
            return ci::vec3(getData(RELATIVE_TILT), getData(RELATIVE_ROTATE), getData(RELATIVE_LATERAL));
        };
        
        MocapDeviceData()
        {
            //init memory
            for(int i=0; i<DEVICE_ARG_COUNT_MAX; i++)
            {
                data[i] = NO_DATA;
            }
            
            for(int i=0; i<4; i++)
                quaternion[i] = NO_DATA;
            
            for(int i=0; i<3; i++ )
                orientationMatrix[i] = NO_DATA;
            
            device = MocapDevice::WIIMOTE;
            isAccelScaled = false;
        }
    
protected:
    MocapDevice device; // which device

    
};
    
//assumes syntien
class IPhoneDeviceData : public MocapDeviceData
{
        
public:
    IPhoneDeviceData() : MocapDeviceData()
    {
        device = MocapDevice::IPHONE;
    }
        
    virtual double getAccelMax(){ return IPHONE_ACCELMAX; };
    virtual double getAccelMin(){ return IPHONE_ACCELMIN; };
};
    
//adjust for your sensor calibration
class NotchDeviceData : public MocapDeviceData
{
    
public:
    NotchDeviceData() : MocapDeviceData()
    {
        device = MocapDevice::NOTCH;
    }
    
    virtual double getAccelMax(){ return NOTCH_ACCELMAX; };
    virtual double getAccelMin(){ return NOTCH_ACCELMIN; };
};
    

        
    
    

    
};
    
#endif /* MotionCaptureData_h */

