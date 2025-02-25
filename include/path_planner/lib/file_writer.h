#ifndef FILE_WRITER_H
#define FILE_WRITER_H

#include <string>
#include <fstream>
#include <lat_long.h>

class FileWriter {
public:
    FileWriter() = delete; //prevent usage of default constructor
    FileWriter(const std::string& base_directory);
    ~FileWriter();
    void writeToFile(const LatLong& latlong);

private:
    std::string directory_path_;
    std::string file_path_;
    std::ofstream file_;
    std::string getCurrentDate();
    std::string getCurrentDateTime();
};

#endif // FILE_WRITER_H
