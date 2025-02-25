#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include <string>
#include <fstream>
#include <lat_long.h>

class CSVWriter {
public:
    CSVWriter() = delete; //prevent usage of default constructor
    CSVWriter(const std::string& base_directory);
    ~CSVWriter();
    void writeToFile(const LatLong& latlong);

private:
    std::string directory_path_;
    std::string file_path_;
    std::ofstream file_;
    std::string getCurrentDate();
    std::string getCurrentDateTime();
};

#endif // CSV_WRITER_H
