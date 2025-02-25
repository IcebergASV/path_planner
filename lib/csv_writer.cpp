#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <csv_writer.h>

namespace fs = std::filesystem;

CSVWriter::CSVWriter(const std::string& base_directory)
{
    // Get the current date to create the directory
    std::string date = getCurrentDate();
    directory_path_ = base_directory + "/" + date;

    // Create the directory if it doesn't exist
    if (!fs::exists(directory_path_)) {
        fs::create_directories(directory_path_);
    }

    // Generate the file name using current date and time
    file_path_ = directory_path_ + "/" + getCurrentDateTime() + ".csv";

    // Open the file for writing
    file_.open(file_path_, std::ios::out);
    if (!file_) {
        throw std::ios_base::failure("Failed to open file: " + file_path_);
    }

    // Write the CSV headers
    file_ << "latitude,longitude" << std::endl;
}

CSVWriter::~CSVWriter()
{
    if (file_.is_open()) {
        file_.close();
    }
}

void CSVWriter::writeToFile(const LatLong& latlong)
{
    if (file_.is_open()) {
        std::cout << "latitude: " << latlong.latitude << "\n";
        file_ << std::fixed << std::setprecision(8) << latlong.latitude << "," << latlong.longitude << std::endl;
    } else {
        std::cerr << "File is not open!" << std::endl;
    }
}

std::string CSVWriter::getCurrentDate() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::tm tm = *std::localtime(&in_time_t);
    char buf[80];
    strftime(buf, sizeof(buf), "%Y_%m_%d", &tm);
    return std::string(buf);
}

std::string CSVWriter::getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::tm tm = *std::localtime(&in_time_t);
    char buf[80];
    strftime(buf, sizeof(buf), "%H_%M_%S", &tm);
    return std::string(buf);
}

