#include <iostream>
#include "../include/path_planner/lib/file_writer.h"
 int main() {
    try {
        // Example usage of the FileWriter class
        FileWriter writer("/home/parallels/ros2_ws/src/path_planner/tests", ":");  // Specify your base directory and delimiter
        
        writer.writeToFile("Hello");
        writer.writeToFile("World");
        writer.writeToFile("This is a test string.");

    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}