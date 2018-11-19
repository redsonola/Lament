//
//  ReadCSV.h
//  ReadCSVClass
//
//  Created by courtney on 8/15/14.
//
//

#ifndef ReadCSVClass_ReadCSV_h
#define ReadCSVClass_ReadCSV_h

#include <sstream>
#include <fstream>


/*
 //use case
 ReadCSV rCSV(filename);
 rCSV.getLine();

 while( !rCSV.eof() && rCSV.correctFormat() )
 {
    std::vector<char *> rCSV.getTokensLine();
    processToken()
 }
 
 rCSV.close();
 
 */

const int BUF_NUM = 1024 * 4;
class ReadCSV
{
public:
    ReadCSV();
    ReadCSV(std::string filename);
    void init(std::string filename);
    inline bool opened(){ return open; };
    inline bool eof(){ return fstr.eof(); };
    inline void close(){ fstr.close(); };
    std::vector<std::string> getTokensInLine();

    
private:
    std::fstream fstr;
    bool open;
};

ReadCSV::ReadCSV(std::string filename)
{
    init(filename);
}

ReadCSV::ReadCSV()
{
    //NOTHING! NOT INIT!
}

void ReadCSV::init(std::string filename)
{
    fstr.open(filename, std::ios::in);
    open = fstr.is_open();
}

std::vector<std::string> ReadCSV::getTokensInLine()
{
    assert(opened());
    
    if (!opened())
    {
        std::abort(); 
    }
    
    std::vector<std::string> tokens;
    int lastComma, fileIndex;
    std::string data;
    char str[BUF_NUM];
    
    fstr.getline(str, BUF_NUM);
    data.assign(str);

    //do again if only these characters
    if (!data.compare("\n") && !data.compare("\r"))
    {
        fstr.getline(str, BUF_NUM);
        data.assign(str);
    }
    
    lastComma = data.find_first_of(",");
    if (lastComma == -1)
    {
        std::cout << "Incorrect Format\n";
        return tokens; //get out
    }
    fileIndex = lastComma;
    tokens.push_back(data.substr(0, lastComma));
    while (fileIndex !=-1)
    {
        fileIndex = data.find(",", lastComma + 1);
        if (fileIndex != -1)
                tokens.push_back(data.substr(lastComma + 1, fileIndex - 1 - lastComma));
        else tokens.push_back(data.substr(lastComma + 1, data.length() - lastComma - 1));
        lastComma = fileIndex;
    }

    return tokens;
}

#endif
