#ifndef FILE_WRITER_H
#define FILE_WRITER_H

#include <string>
#include <fstream>

class FileWriter {
public:
    FileWriter() = delete; //prevent usage of default constructor
    FileWriter(const std::string& base_directory, const std::string& delimiter);
    ~FileWriter();
    void writeToFile(const std::string& data);

private:
    std::string directory_path_;
    std::string file_path_;
    std::ofstream file_;
    std::string delimiter_;
    std::string getCurrentDate();
    std::string getCurrentDateTime();
};

#endif // FILE_WRITER_H
