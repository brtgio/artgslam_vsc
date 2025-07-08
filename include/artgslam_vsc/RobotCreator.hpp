#pragma once

#include <iostream>
#include <SFML/Graphics.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/Widgets/EditBox.hpp>
#include <TGUI/Widgets/Button.hpp>
#include <ros/ros.h>
#include <regex>
#include <cctype>

#include "artgslam_vsc/UnicicleWmr.hpp"

class RobotCreator
{
public:
    // Constructor explícito para evitar conversiones accidentales
    explicit RobotCreator(UnicicleWmr& wmrRef);

    // Desactivar constructor por defecto
    RobotCreator() = delete;

    RobotCreator(const RobotCreator&) = delete;
    RobotCreator& operator=(const RobotCreator&) = delete;

    void run();
    void update();
    void processEvents();
    void render();
    void setupWidgets();
    void setupCallbacks();
    bool isRunning() const;

private:
    sf::RenderWindow windowRobotCreator;
    tgui::Gui gui;
    UnicicleWmr& wmr;

    // Widgets del formulario
    tgui::EditBox::Ptr widthBox;
    tgui::EditBox::Ptr heightBox;
    tgui::EditBox::Ptr colorBox;

    tgui::Button::Ptr resetButton;
    tgui::Button::Ptr createButton;

    // Funciones auxiliares
    bool isValidFloat(const std::string& str);
    bool isValidHexColor(const std::string& str);
    sf::Color hexToColor(const std::string& hex);
};
