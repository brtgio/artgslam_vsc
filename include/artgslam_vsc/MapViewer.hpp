#pragma once

#include "MenuBar.hpp"
#include "FileManager.hpp"
#include "GridMap.hpp"
#include "RosHandler.hpp"
#include "ViewController.hpp"
#include "artgslam_vsc/RobotCreator.hpp"
#include "artgslam_vsc/UnicicleWmr.hpp"
#include "artgslam_vsc/RosHandler.hpp"
#include "artgslam_vsc/LiveMap.hpp"
#include "artgslam_vsc/GridMap.hpp"
#include <SFML/Graphics.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>


class MapViewer {
private:
    sf::RenderWindow& window;
    
    sf::View view;
    tgui::Gui gui;

    MenuBar menu;
    FileManager manager;
    RosHandler roshandler;
    ViewController controller;
    GridMap map;
    UnicicleWmr wmr;
    LiveMap livemap;

    bool running = true;

public:
    // Constructor necesario
    MapViewer(sf::RenderWindow& win);

    void update();
    void processEvent();
    void render();

    [[nodiscard]] bool isRunning() const;
};
