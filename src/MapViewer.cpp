#include "artgslam_vsc/MapViewer.hpp"
#include <sstream>
#include <iomanip>

MapViewer::MapViewer(sf::RenderWindow& win)
    : window(win)
    , view(window.getDefaultView())
    , gui(win)
    , controller(win, 0.1f, 50.0f, view)
    , map(100, 0.1, controller)
    , manager(map, controller)
    , menu(gui)
    , roshandler()
    , wmr()
    , livemap(100, 0.1, controller)
{
    window.setFramerateLimit(60);

    menu.setCallbacks(
        [this]() { manager.loadDialog(); },
        [this]() { manager.saveDialog(); },
        [this]() { /* handleSaveImage */ },
        [this]() { running = false; window.close(); },
        [this]() { controller.reset(); },
        [this]() { map.clearGridMap(); },
        [this]() {
            RobotCreator creator(wmr);
            creator.run();
        },
        [this]() {}
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

    float worldX_m = worldPos.x / pixelsPerMeter;
    float worldY_m = worldPos.y / pixelsPerMeter;

    int i = map.getCellIndexY(worldY_m);
    int j = map.getCellIndexX(worldX_m);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "Grid: (" << cellX << ", " << cellY << "), Real: ("
        << worldX_m << ", " << worldY_m << "), Index: ["
        << i << ", " << j << "]";

    menu.updateCoordinates(oss.str());

    if (menu.getLiveMode()) {
        livemap.clearPoints();
        livemap.clearGrid();

        double v = roshandler.getLinearVelocity();
        double w = roshandler.getAngularVelocity();
        wmr.setVelocity(v, w);

        const auto& sonar = roshandler.getSonarPoints();
        for (const auto& p : sonar) {
            livemap.addPoint(p.x, p.y);
        }

        livemap.updateGridFromPoints();
    }

    wmr.update(roshandler.getlast_dt());
}

void MapViewer::processEvent()
{
    sf::Event event;
    while (window.pollEvent(event)) {
        gui.handleEvent(event);
        controller.handleEvent(event);

        if (event.type == sf::Event::Closed) {
            running = false;
            window.close();
        }
    }
}

void MapViewer::render()
{
    window.clear(sf::Color::Black);
    controller.applyView();  // Aplica vista con zoom y pan

    controller.drawGrid(window);   // Dibuja rejilla primero
    controller.drawAxes(window);   // Dibuja ejes

    if (menu.getLiveMode()) {
        const auto& gridLive = livemap.getGrid();
        if (gridLive.empty() || gridLive[0].empty()) {
            std::cout << "Live grid is empty!" << std::endl;
            window.setView(window.getDefaultView());
            gui.draw();
            window.display();
            return;
        }
        livemap.drawLiveMap(window);  // Dibuja mapa sobre la rejilla
    } else {
        const auto& gridMap = map.getGrid();
        if (gridMap.empty() || gridMap[0].empty()) {
            std::cout << "Grid map is empty!" << std::endl;
            window.setView(window.getDefaultView());
            gui.draw();
            window.display();
            return;
        }
        map.draw(window, controller.getPixelsPerMeter());  // Igual
    }

    wmr.draw(window);

    window.setView(window.getDefaultView());
    gui.draw();
    window.display();
}


bool MapViewer::isRunning() const {
    return running && window.isOpen();
}
