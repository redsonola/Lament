//
//  MelodyGeneratorAlgorithm.h
//  MagneticGardel
//
//  Created by courtney on 11/16/17.
//
//

#ifndef MelodyGeneratorAlgorithm_h
#define MelodyGeneratorAlgorithm_h
namespace CRCPMotionAnalysis {
    
    class MelodyGeneratorAlgorithm
    {
    private:
        bool trained;
    protected:
        float bpm;
        double tpb;
        std::string dbfile;
    public:
        MelodyGeneratorAlgorithm()
        {
            trained = false;
        };
    
        virtual void train(std::string _dbfileName, int track = 1)
        {
            trained = true;
            dbfile = _dbfileName;
        };

        std::string getFile()
        {
            return dbfile; 
        }

        virtual MidiNote generateNext()
        {
            
            
        };
        
        virtual bool isTrained()
        {
            return trained;
        }
        
        virtual float getBPM()
        {
            return bpm;
        }
        virtual double getTicksPerBeat()
        {
            return tpb;
        }

    };
    
    class PST : public MelodyGeneratorAlgorithm
    {
        PST()
        {
            
        }
    };
}

#endif /* MelodyGeneratorAlgorithm_h */
