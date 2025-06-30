#include "include/GridMap.hpp"

GridMap::GridMap(int size, double resolution)
    : gridSize(size), gridResolution(resolution)
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

void GridMap::setPoints(const std::vector<double> &newX, const std::vector<double> &newY)
{
    posX = newX;
    posY = newY;
}

void GridMap::clearPoints()
{
    posX.clear();
    posY.clear();
}

void GridMap::xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                      std::vector<int>& xGrid, std::vector<int>& yGrid)
{
const double resolution = 0.1;
    const int gridSize = 1000; // Mapa de 1000x1000 celdas
    const int halfGrid = gridSize / 2;

    size_t n = std::min(x.size(), y.size());
    xGrid.resize(n);
    yGrid.resize(n);

    for (size_t i = 0; i < n; ++i) {
        int gx = static_cast<int>(std::round(x[i] / resolution));
        int gy = static_cast<int>(std::round(y[i] / resolution));

        // Centrar el origen en el medio del grid
        xGrid[i] = gx + halfGrid;
        yGrid[i] = gy + halfGrid;
    }
}

void GridMap::fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid)
{
        const int gridSize = 1000;

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
