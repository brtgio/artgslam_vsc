#include "artgslam_vsc/MapViewer.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>

MapViewer::MapViewer(sf::RenderWindow& win)
    : window(win)
    , view(window.getDefaultView())
    , gui(win)
    , controller(win, 0.1f, 50.0f, view)
    , map(1000, 0.1, controller)
    , manager(map, controller)
    , menu(gui)
    , roshandler()
    , wmr()
    , livemap(1000, 0.1, controller)
    ,r_menu(gui,map,livemap)
{
    r_menu.connectSignals();
    window.setFramerateLimit(120);

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
    int gridSize = map.getMapSize();
    // 1. Obtener índice de celda bajo el mouse (columna, fila)
    sf::Vector2i gridIndex = controller.getHoveredCell(gridSize);

    // 2. Posición del mouse en el mundo (metros, no píxeles)
    sf::Vector2f worldPos = controller.getMouseWorldPosition();

    // 3. Mostrar info (solo si está dentro del mapa)
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "Mouse: (" << worldPos.x << ", " << worldPos.y << ") m";
    
    if (gridIndex.x != -1 && gridIndex.y != -1) {
        oss << " | Grid: (" << gridIndex.x << ", " << gridIndex.y << ")";
    } else {
        oss << " | Fuera del mapa";
    }

    menu.updateCoordinates(oss.str());

    // -----------------------------------------------
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
    while (window.pollEvent(event))
    {
        gui.handleEvent(event);
        controller.handleEvent(event);

        if (event.type == sf::Event::Closed) {
            running = false;
            window.close();
        }

        if (event.type == sf::Event::MouseButtonPressed)
        {
            // 1) Refresca el índice AQUÍ, en el momento del clic
            const int gridSize = map.getMapSize();            // o map.cols()
            const sf::Vector2i cell = controller.getHoveredCell(gridSize);

            const sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
            const sf::Vector2f pixelPosF(pixelPos);

            if (event.mouseButton.button == sf::Mouse::Right) {
                // 2) Pasa el índice recién obtenido, no el de update()
                r_menu.show(static_cast<float>(pixelPos.x),
                            static_cast<float>(pixelPos.y),
                            cell);
                r_menu.setVisible(true);
            }
            else if (event.mouseButton.button == sf::Mouse::Left) {
                if (r_menu.isVisible() && !r_menu.containsPoint(pixelPosF)) {
                    r_menu.setVisible(false);
                }
            }
        }
    }
}



void MapViewer::render()
{
    window.clear(sf::Color(0,0,0,255));
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
