#pragma once

#include "MenuBar.hpp"
#include "FileManager.hpp"
#include "GridMap.hpp"
#include "ViewController.hpp"
#include <SFML/Graphics.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>

class MapViewer {
private:
    sf::RenderWindow& window;
    sf::View view;
    tgui::Gui gui;

    MenuBar menu;
    FileManager manager;
    ViewController controller;
     GridMap& map;
    bool running = true;

public:
    // Constructor necesario
    MapViewer(sf::RenderWindow& win, GridMap& sharedMap);

    void update();
    void processEvent();
    void render();

    [[nodiscard]] bool isRunning() const;
};
