#pragma once

#include "artgslam_vsc/MenuBar.hpp"
#include "artgslam_vsc/FileManager.hpp"
#include "artgslam_vsc/GridMap.hpp"
#include "artgslam_vsc/RosHandler.hpp"
#include "artgslam_vsc/ViewController.hpp"
#include "artgslam_vsc/RobotCreator.hpp"
#include "artgslam_vsc/UnicicleWmr.hpp"
#include "artgslam_vsc/LiveMap.hpp"
#include "artgslam_vsc/RightClickMapMenu.hpp"
#include "artgslam_vsc/AStar.hpp"
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
    RightClickMapMenu r_menu;
    AStar aStarsim;
    
    

    bool running = true;
    sf::Vector2f worldXY;
    sf::Vector2i gridIndex;
    sf::Vector2i gridIndex2copy;

    bool astarAnimating = false;
    bool astarCompleted = false;

public:
    // Constructor necesario
    MapViewer(sf::RenderWindow& win);

    void update();
    void processEvent();
    void render();

    [[nodiscard]] bool isRunning() const;
};
