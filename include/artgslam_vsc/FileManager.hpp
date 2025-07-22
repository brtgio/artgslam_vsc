#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>  // para std::setprecision
#include "GridMap.hpp"
#include "ViewController.hpp"
#include "tinyfiledialogs.h"
class FileManager
{
private:
    GridMap& map;
    std::string loadedFilename;  // Guarda el archivo seleccionado
public:
    FileManager(GridMap& mapRef);
    void loadDialog();
    void saveDialog();
    void dataLoad(const std::string& filename, std::vector<double>& x, std::vector<double>& y);
    void saveData(const std::string& filename, std::vector<double>& x, std::vector<double>& y);
    void saveScreen(const std::string& filename);
};



