#include "FileManager.hpp"
 // Instancia global o estática de GridMap

FileManager::FileManager(GridMap& mapRef,ViewController& controllerRef)
 : map(mapRef) ,controller(controllerRef) 
{
}

void FileManager::loadDialog()
{
    const char* path = tinyfd_openFileDialog(
        "Open Dataset",
        "",            // Default path
        0,             // No filter patterns
        nullptr,       // No filter extensions
        nullptr,       // No filter description
        0              // Single selection
    );

    if (path) {
        std::cout << "Archivo seleccionado: " << path << std::endl;

        loadedFilename = path;  // Guardar el path seleccionado

        // Cargar datos en vectores locales
        std::vector<double> xData, yData;
        dataLoad(loadedFilename, xData, yData);

        // Actualizar puntos en el mapa
        map.setPoints(xData, yData);

        // Actualizar la grilla con los nuevos puntos cargados
        std::vector<int> xGrid, yGrid;
        map.xy2Grid(map.getRealX(), map.getRealY(), xGrid, yGrid);
        map.fillGrid(xGrid, yGrid);

    } 
    else {
        std::cout << "No se seleccionó ningún archivo." << std::endl;
    }
}

void FileManager::saveDialog()
{
    const char* path = tinyfd_saveFileDialog(
        "Save File",
        "output.txt",
        0,
        nullptr,
        nullptr
    );

    if (path) {
        std::cout << "Saving to: " << path << std::endl;
        // Llamar a saveData pasándole el path y vectores del mapa
        saveData(path, const_cast<std::vector<double>&>(map.getRealX()), const_cast<std::vector<double>&>(map.getRealY()));
    } else {
        std::cout << "Save operation was canceled.\n";
    }
}

void FileManager::dataLoad(const std::string& filename, std::vector<double>& x, std::vector<double>& y)
{
    x.clear();
    y.clear();

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::string line;
    size_t count = 0;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        // Separar la línea en tokens separados por coma
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.size() >= 2) {
            try {
                double xVal, yVal;

                if (tokens.size() == 2) {
                    // Formato: x,y
                    xVal = std::stod(tokens[0]);
                    yVal = std::stod(tokens[1]);
                } else {
                    // Formato >=3 columnas: ignorar la 1ª, tomar 2ª y 3ª como x,y
                    xVal = std::stod(tokens[1]);
                    yVal = std::stod(tokens[2]);
                }

                x.push_back(xVal);
                y.push_back(yVal);
                ++count;
            } catch (const std::exception& e) {
                std::cerr << "Error parsing line: " << line << " (" << e.what() << ")\n";
            }
        } else {
            std::cerr << "Formato inválido en línea (menos de 2 columnas): " << line << "\n";
        }
    }

    std::cout << "Datos cargados: " << count << " puntos\n";
}

void FileManager::saveData(const std::string &filename, std::vector<double> &x, std::vector<double> &y)
{
    if (x.size() != y.size()) {
        std::cerr << "Error: posX and posY sizes do not match.\n";
        return;
    }

    std::ofstream outFile(filename);
    if (!outFile) {
        std::cerr << "Error: Cannot open file for writing.\n";
        return;
    }

    outFile << std::fixed << std::setprecision(17);
    for (size_t i = 0; i < x.size(); ++i) {
        outFile << x[i] << "," << y[i] << "\n";
    }

    outFile.close();
    std::cout << "txt file saved successfully.\n";
}

void FileManager::saveScreen(const std::string &filename)
{
    const char* path = tinyfd_saveFileDialog(
            "Save Image",
            "Map.png",
            0,
            nullptr,
            nullptr
        );
        if (path) {
            
            return;
        }
}
