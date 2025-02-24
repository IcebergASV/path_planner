#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include "../include/path_planner/lib/file_writer.h"

namespace fs = std::filesystem;

FileWriter::FileWriter(const std::string& base_directory, const std::string& delimiter)
    : delimiter_(delimiter) 
{
    // Get the current date to create the directory
    std::string date = getCurrentDate();
    directory_path_ = base_directory + "/" + date;

    // Create the directory if it doesn't exist
    if (!fs::exists(directory_path_)) {
        fs::create_directories(directory_path_);
    }

    // Generate the file name using current date and time
    file_path_ = directory_path_ + "/" + getCurrentDateTime() + ".txt";

    // Open the file for writing
    file_.open(file_path_, std::ios::out | std::ios::app);
    if (!file_) {
        throw std::ios_base::failure("Failed to open file: " + file_path_);
    }
}

FileWriter::~FileWriter()
{
    if (file_.is_open()) {
        file_.close();
    }
}

void FileWriter::writeToFile(const std::string& data)
{
    if (file_.is_open()) {
        file_ << data << delimiter_;
    } else {
        std::cerr << "File is not open!" << std::endl;
    }
}

std::string FileWriter::getCurrentDate() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::tm tm = *std::localtime(&in_time_t);
    char buf[80];
    strftime(buf, sizeof(buf), "%Y_%m_%d", &tm);
    return std::string(buf);
}

std::string FileWriter::getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::tm tm = *std::localtime(&in_time_t);
    char buf[80];
    strftime(buf, sizeof(buf), "%H_%M_%S", &tm);
    return std::string(buf);
}

