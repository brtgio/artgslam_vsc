#pragma once
#include <SFML/Graphics.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/AllWidgets.hpp>
#include <ros/package.h>
#include <iostream>
#include <regex>

class RightClickMapMenu {
public:
    RightClickMapMenu(tgui::Gui& guiRef);

    void setupWiggets();

    void show(float x, float y);
    void hide();
    bool isVisible() const { return visible; }
    void setVisible(bool show);
    void connectSignals();

private:
    tgui::Gui& gui;
    tgui::Group::Ptr container;
    tgui::ChildWindow::Ptr panel;
    tgui::Button::Ptr startBtn;
    tgui::Button::Ptr goalBtn;
    tgui::Button::Ptr clearBtn;
    tgui::Button::Ptr clearViewBtn;
    bool visible = false;
};
