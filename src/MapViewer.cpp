#include "artgslam_vsc/MapViewer.hpp"

MapViewer::MapViewer(sf::RenderWindow& win, GridMap& sharedMap)
    : window(win)
    , gui(win)
    , view(view)                     
    , controller(win, 0.5f, 2.0f, view)              
    , manager(sharedMap, controller)                
    , map(sharedMap)
    , menu(gui)
{

    window.setFramerateLimit(60);

    menu.setCallbacks(
        [this]() { manager.loadDialog(); },
        [this]() { manager.saveDialog(); },  // o manager.saveData()
        [this]() { /* handleSaveImage */ },
        [this]() { running = false; window.close(); },
        [this]() { controller.reset(); },    // Delegamos a ViewController
        [this]() { map.clearGridMap(); } // si implementas
    );
}

void MapViewer::update()
{
    sf::Vector2i pixelPos = controller.getMousePixelPosition();
    sf::Vector2f worldPos = controller.getMouseWorldPosition();

    float metersPerCell = controller.getMetersPerCell();
    float pixelsPerMeter = controller.getPixelsPerMeter();

    int cellX = static_cast<int>(std::floor(worldPos.x / (metersPerCell * pixelsPerMeter)));
    int cellY = static_cast<int>(std::floor(worldPos.y / (metersPerCell * pixelsPerMeter)));
    float worldX_m = worldPos.x / controller.getPixelsPerMeter();
float worldY_m = worldPos.y / controller.getPixelsPerMeter();

    int i = map.getCellIndexY(worldY_m);  // fila
    int j = map.getCellIndexX(worldX_m);  // columna
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Grid: (" << cellX << ", " << cellY << "), Real: ("
    << worldX_m << ", " << worldY_m << "), Index: [" << i << ", " << j << "]";


    menu.updateCoordinates(oss.str());  // ✅ Esto actualiza el texto en pantalla
}

void MapViewer::processEvent()
{
    sf::Event event;
    while (window.pollEvent(event))
    {
        gui.handleEvent(event);         // Eventos de GUI
        controller.handleEvent(event);  // Paneo, zoom, reset

        if (event.type == sf::Event::Closed)
        {
            running = false;
            window.close();
        }
    }
}

void MapViewer::render()
{
    window.clear(sf::Color::Black);
    controller.applyView();

    const auto& gridMap = map.getGrid();
    if (gridMap.empty() || gridMap[0].empty()) {
        std::cout << "Grid map is empty!" << std::endl;
        window.setView(window.getDefaultView());
        gui.draw();
        window.display();
        return;
    }

    // Obtener dimensiones
    int rows = static_cast<int>(gridMap.size());
    int cols = static_cast<int>(gridMap[0].size());

    // Obtener parámetros de escala
    float zoom = controller.getZoom();
    float cellSize = 0.1f * 200.f * zoom;  // metersPerCell * pixelsPerMeter * zoom

    // Centro del mapa en (0,0)
    float mapWidth = cols * cellSize;
    float mapHeight = rows * cellSize;
    float offsetX = -mapWidth / 2.f;
    float offsetY = -mapHeight / 2.f;

    sf::RectangleShape cellShape(sf::Vector2f(cellSize, cellSize));
    cellShape.setFillColor(sf::Color::Yellow);

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (gridMap[row][col] == 1) {
                float x = offsetX + col * cellSize;
                float y = offsetY + row * cellSize;

                cellShape.setPosition(x, y);
                window.draw(cellShape);
            }
        }
    }

    controller.drawGrid(window);
    controller.drawAxes(window);

    window.setView(window.getDefaultView());
    gui.draw();
    window.display();
}




bool MapViewer::isRunning() const
{
    return running && window.isOpen();
}

