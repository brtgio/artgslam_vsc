#pragma once

#include <SFML/Graphics.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/AllWidgets.hpp>
#include <ros/package.h>
#include <iostream>
#include <regex>
#include "artgslam_vsc/GridMap.hpp"
#include "artgslam_vsc/LiveMap.hpp"

class RightClickMapMenu {
public:
    RightClickMapMenu(tgui::Gui& guiRef, GridMap &mapRef,LiveMap& livemapRef );

    void setupWidgets();

    // Mostrar menú en posición (x, y) y guardar coordenadas del clic
    void show(float x, float y, const sf::Vector2i& gridIndex);
    void hide();
    void setVisible(bool show);
    bool isVisible() const;

    void connectSignals();

    // Nuevo método para saber si un punto está dentro del panel
    bool containsPoint(const sf::Vector2f& point) const;
    bool isSet() const{return isGoalActive && isStartActive;};

private:
    // Referencia a la GUI de TGUI
    tgui::Gui& gui;
    GridMap& gridmap;
    LiveMap& livemap;

    // Contenedor del formulario y widgets
    tgui::Group::Ptr container;
    tgui::ChildWindow::Ptr panel;
    tgui::Button::Ptr start;
    tgui::Button::Ptr goal;
    tgui::Button::Ptr clear;
    tgui::Button::Ptr clearView;

    // Estado de visibilidad
    bool visible = false;

    // Copias de coordenadas congeladas al momento del clic
    sf::Vector2f worldXY_copy;
    sf::Vector2i gridIndex_copy;
    sf::Vector2i startIndex;
    sf::Vector2i goalIndex;

    //variables de acrivacion
    bool isGoalActive = false;
    bool isStartActive = false;
};
