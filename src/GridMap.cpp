#include "artgslam_vsc/GridMap.hpp"
#include "artgslam_vsc/AStar.hpp" 
#include <iostream>


GridMap::GridMap(int size, double resolution, ViewController& controller)
    : gridSize(size), gridResolution(resolution), controller(controller)
{
    // Inicializamos el grid vacío de una vez
    grid.assign(gridSize, std::vector<int>(gridSize, 0));
}

int GridMap::getCellIndexX(double realX) const
{
    double offset = (gridSize * gridResolution) / 2.0;  // si tu origen está al centro
    int j = static_cast<int>(std::floor((realX + offset) / gridResolution));
    if (j < 0) j = 0;
    if (j >= gridSize) j = gridSize - 1;
    return j;
}

int GridMap::getCellIndexY(double realY) const
{
    double offset = (gridSize * gridResolution) / 2.0;  // igual al eje Y
    int i = static_cast<int>(std::floor((realY + offset) / gridResolution));
    if (i < 0) i = 0;
    if (i >= gridSize) i = gridSize - 1;
    return i;
}

void GridMap::addPoints(double x, double y)
{
    posX.push_back(x);
    posY.push_back(y);
}

void GridMap::setPoints(const std::vector<double>& newX, const std::vector<double>& newY)
{
    posX = newX;
    posY = newY;
}

void GridMap::clearPoints()
{
    posX.clear();
    posY.clear();
}

void GridMap::setStart(int col, int row)
{
    std::cout << "[setStart] col=" << col << " row=" << row << '\n';
    if (grid[row][col] == 0){
        grid[row][col] = 's'; // valor ASCII para start
        std::cout << "Se guardó el valor de " << grid[row][col] << std::endl;
    }
    else{
        std::cout << "[setStart] Celda ocupada." << std::endl;
    }
}

void GridMap::setGoal(int col, int row)
{
    std::cout << "[setGoal] col=" << col << " row=" << row << '\n';
    if (grid[row][col] == 0){
        grid[row][col] = 'g'; // valor ASCII para goal
        std::cout << "Se guardó el valor de " << grid[row][col] << std::endl;
    }
    else{
        std::cout << "[setGoal] Celda ocupada." << std::endl;
    }
}

int GridMap::isOccupied(int i, int j) const
{
    if(grid[j][i]==1){
        return 1;
    }
    if(grid[j][i]==0){
        return 0;
    }
    if(grid[j][i]=='s'){
        return 's';
    }
    if(grid[j][i]=='g'){
        return 'g';
    }
    else{
        return 1;
    }
}
void GridMap::xy2Grid(const std::vector<double> &x, const std::vector<double> &y,
                      std::vector<int> &xGrid, std::vector<int> &yGrid)
{
    const int halfGrid = gridSize / 2;
    size_t n = std::min(x.size(), y.size());
    xGrid.resize(n);
    yGrid.resize(n);

    for (size_t i = 0; i < n; ++i) {
        int gx = static_cast<int>(std::round(x[i] / gridResolution));
        int gy = static_cast<int>(std::round(y[i] / gridResolution));

        // Centrar el origen en el medio del grid
        xGrid[i] = gx + halfGrid;
        yGrid[i] = gy + halfGrid;
    }
}

void GridMap::fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid)
{
    grid.assign(gridSize, std::vector<int>(gridSize, 0));

    for (size_t i = 0; i < xGrid.size(); ++i) {
        int x = xGrid[i];
        int y = yGrid[i];

        if (x >= 0 && x < gridSize && y >= 0 && y < gridSize) {
            grid[y][x] = 1;
        }
    }
}

const std::vector<std::vector<int>>& GridMap::getGrid() const
{
    return grid;
}

void GridMap::clearGridMap()
{
    grid.assign(gridSize, std::vector<int>(gridSize, 0));
}

void GridMap::clearSetPoints(sf::Vector2i cellIndex)
{
    if (cellIndex.x >= 0 && cellIndex.y >= 0 &&
        cellIndex.y < gridSize && cellIndex.x < gridSize) {
        grid[cellIndex.y][cellIndex.x] = 0;
    }
}

void GridMap::draw(sf::RenderTarget &target, float pixelsPerMeter) const
{
    if (grid.empty() || grid[0].empty()) return;

    const int rows = static_cast<int>(grid.size());
    const int cols = static_cast<int>(grid[0].size());

    const float cellSize = gridResolution * pixelsPerMeter;

    // Centrado en (0,0): desplazamiento hacia la esquina superior izquierda
    const float offsetX = -(cols / 2.f) * cellSize;
    const float offsetY = -(rows / 2.f) * cellSize;

    sf::RectangleShape cellShape({cellSize, cellSize});

    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            const int val = grid[row][col];

            // Solo celdas marcadas
            if (val == 1)
                cellShape.setFillColor(sf::Color::Yellow);
            else if (val == 's')
                cellShape.setFillColor(sf::Color::Red);
            else if (val == 'g')
                cellShape.setFillColor(sf::Color::Green);
            else
                continue;

            // Posición en mundo
            const float x = offsetX + col * cellSize;
            const float y = offsetY + row * cellSize;

            cellShape.setPosition(x, y);
            target.draw(cellShape);
        }
    }
}