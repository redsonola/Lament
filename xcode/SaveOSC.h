//
//  SaveOSCToCSV.h
//  VideoAndOSCLab
//
//  Created by courtney on 11/11/18.
//

#ifndef SaveOSCToCSV_h
#define SaveOSCToCSV_h

#include <iostream>
#include <fstream>

namespace CRCPMotionAnalysis {
    
   class SaveOSC
    {
    protected:
        std::string filename;
    public:
        SaveOSCToCSV(filename_)
        {
            filename = filename_
        };
        
        void add(ci::osc::Message msg, seconds);
        {
            ofstream oscfile;
            myfile.open (filename, ios::out | ios::app );
            
        }
        

    };
}


#endif /* SaveOSCToCSV_h */
