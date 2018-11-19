//
//  SaveOSCToCSV.h
//  VideoAndOSCLab
//
//  Created by courtney on 11/11/18.
//

#ifndef SaveOSC_h
#define SaveOSC_h

#include <iostream>
#include <fstream>
#include "ReadCSV.h"

namespace CRCPMotionAnalysis {
    
   class SaveOSC
    {
    protected:
        std::string filename;
    public:
        SaveOSC(std::string filename_)
        {
            filename = filename_;
        };
        
        void add(ci::osc::Message msg, float seconds)
        {
            std::ofstream oscfile;
            oscfile.open (filename, std::ios::out | std::ios::app );
            oscfile << seconds << "," << msg.getAddress() << "," << msg.getTypeTagString(); //note that the typetag string begins with a comma
            
            for(int i=0; i<msg.getNumArgs(); i++) // TODO: add more handlers for different osc types see: http://opensoundcontrol.org/spec-1_0-examples
            {
//                enum class ArgType : char { INTEGER_32 = 'i', FLOAT = 'f', DOUBLE = 'd', STRING = 's', BLOB = 'b', MIDI = 'm', TIME_TAG = 't', INTEGER_64 = 'h', BOOL_T = 'T', BOOL_F = 'F', CHAR = 'c', NULL_T = 'N', IMPULSE = 'I', NONE = NULL_T };

                ci::osc::ArgType type_ = msg.getArgType(i);
                oscfile << "," ;
                switch (type_) {
                    case ci::osc::ArgType::INTEGER_32:
                        oscfile << msg.getArgInt32(i);
                        break;
                    case ci::osc::ArgType::FLOAT:
                        oscfile << msg.getArgFloat(i);
                        break;
                    case ci::osc::ArgType::DOUBLE:
                        oscfile << msg.getArgDouble(i);
                        break;
                    case ci::osc::ArgType::STRING:
                        oscfile << msg.getArgString(i);
                        break;
                    default:
                        break;
                }
            }
            oscfile << std::endl;
            oscfile.close();
        }
    };
    
    //utility class that's really a struct to just allow attaching the message with the timestamp
    class OSCMessageTimeStamp
    {
    public:
        float timeStamp;
        ci::osc::Message msg;
        
        OSCMessageTimeStamp(float stamp, ci::osc::Message m)
        {
            timeStamp = stamp;
            msg = m;
        };
    };
    
    //plays all saved OSC from session and sends it to where we are listening...
    //TODO: any kind of fastforwarding, rewinding, starting in time where the OSC starts should be coded by you
    class PlayOSC
    {
    protected:
        std::string filename;

        
        ReadCSV csvFile;
        float lastTime;
        
        ci::osc::SenderUdp  mSender;
        std::vector<OSCMessageTimeStamp> msgs;

    public:
                                                                                                   // LOCALPORT, DESTHOST, DESTPORT
        PlayOSC( std::string _fname, std::string desthost, int destport, int localport)  : csvFile(filename), mSender(localport, desthost, destport)
        {
            filename = _fname;
            lastTime = 0;
            csvFile.init(filename);
            
            try{
                mSender.bind();
            }
            catch( ci::osc::Exception &e)
            {
                CI_LOG_E( "PlayOSC: Error binding" << e.what() << " val: " << e.value() );
            }
        }
        
        ~PlayOSC()
        {
            csvFile.close();
        }
        
        //creates an OSC message, assuming a format.
        ci::osc::Message createMsg(std::vector<std::string> tokens)
        {
            ci::osc::Message msg;
            msg.setAddress( tokens[1] );
//            std::cout << tokens[1] << std::endl;
            std::string tag = tokens[2];
            
            //only handles what SaveOSC writes
            for(int i=3; i<tokens.size(); i++)
            {
                if(tag[i-3]=='i')
                {
                    int arg = std::atoi(tokens[i].c_str());
                    msg.append(arg);
                }
                else if(tag[i-3]=='f')
                {
                    float arg = std::atof(tokens[i].c_str());
                    msg.append(arg);
                }
                else if(tag[i-3]=='d')
                {
                    double arg = std::atof(tokens[i].c_str()); //this is a hack - fix if this bothers you
                    msg.append(arg);
                }
                else if(tag[i-3]=='s')
                {
                    msg.append(tokens[i]);
                }
            }
            return msg;
        }
        
        //sends buffered OSC with a timestamp >= time in seconds
        void sendBufferedOSC(float seconds)
        {
            for(int i=msgs.size()-1; i>=0; i--) //TODO: make more efficient by using while loop
            {
//                std::cout << msgs[i].timeStamp ;
                if(msgs[i].timeStamp >= seconds)
                {
//                    std::cout << "sending... " << msgs[i].msg.getAddress() << "\n";

                    mSender.send(msgs[i].msg);
                    msgs.erase(msgs.begin() + i);
                }
//                else std::cout << "not sending...\n";
            }
        }
        
        //reads through file, loads OSC messages, and sends the OSC according to the input time in seconds.
        void update(float seconds)
        {
//            std::cout << "updating:" << lastTime << "," << csvFile.eof() << "\n";

            //1. read file, update buffer
            while(!csvFile.eof() && lastTime <= seconds)
            {
//                std::cout << "in here\n";
                std::vector<std::string> tokens = csvFile.getTokensInLine();
                
                if(tokens.size() >=3 )
                {
                    //create OSC message & send when ready
                    lastTime = std::atof(tokens[0].c_str());
                    ci::osc::Message msg = createMsg(tokens);
                
                    msgs.push_back(OSCMessageTimeStamp(lastTime, msg));
                }
            }
            
            //2. send buffered OSC messages
            sendBufferedOSC(seconds);
        }
        
    };
};


#endif /* SaveOSCToCSV_h */
