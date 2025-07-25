/* 
 * -----------------------------------------------------------------------------
 *  Copyright (c) 2025 Gilberto Ramos Valenzuela
 *
 *  This file is part of the Artgslam Visualizer project.
 *
 *  Licensed for personal, academic, and non-commercial use only.
 *  Commercial use of the complete application "Artgslam Visualizer" is prohibited
 *  without explicit permission from the copyright holder.
 *
 *  For full license details, see the LICENSE.txt file distributed with this software.
 * -----------------------------------------------------------------------------
 */

#include "artgslam_vsc/MenuBar.hpp"
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

/**
 * @brief Constructs the MenuBar and initializes all components.
 * 
 * Creates the main menu bar, the coordinate label, and the live mode toggle button.
 * 
 * @param gui Reference to the TGUI GUI instance where widgets will be added.
 */
MenuBar::MenuBar(tgui::Gui& gui) {
    setupMenu(gui);        /**< Create the main menu bar */
    setupCordLable(gui);  /**< Create coordinate display label */
    setupliveToggle(gui);  /**< Create toggle button for live mode */
}

/**
 * @brief Creates and configures the main menu bar with categorized menu items.
 * 
 * The menu includes File, View, Create Object, and Simulation categories with their items.
 * The menu bar is positioned at the bottom and menus open upward.
 * 
 * @param gui Reference to the TGUI GUI instance.
 */
void MenuBar::setupMenu(tgui::Gui& gui) {
    menuBar = tgui::MenuBar::create();
    menuBar->setSize("100%", 20);
    menuBar->setPosition(0, "100% - 20");  // Bottom of the window
    menuBar->setInvertedMenuDirection(true); // Menus open upward
    gui.add(menuBar);

    // File menu and items
    menuBar->addMenu("File");
    menuBar->addMenuItem("File", "Open");
    menuBar->addMenuItem("File", "Save");
    menuBar->addMenuItem("File", "Save2Image");
    menuBar->addMenuItem("File", "Close");

    // View menu and items
    menuBar->addMenu("View");
    menuBar->addMenuItem("View", "ResetView");
    menuBar->addMenuItem("View", "ClearView");

    // Create object menu and items
    menuBar->addMenu("Create object");
    menuBar->addMenuItem("Create object", "WMR");

    // Simulation menu and items
    menuBar->addMenu("Simulation");
    menuBar->addMenuItem("Simulation", "A*");
}

/**
 * @brief Sets up the coordinate label displayed at the bottom right of the window.
 * 
 * The label initially shows placeholder text and has transparent background with black text.
 * 
 * @param gui Reference to the TGUI GUI instance.
 */
void MenuBar::setupCordLable(tgui::Gui& gui) {
    coordLabel = tgui::Label::create("Grid: -, Real: -");
    coordLabel->setTextSize(14);
    coordLabel->setPosition("100% - 400", "100% - 20"); // Bottom right, offset leftwards
    coordLabel->getRenderer()->setBackgroundColor(tgui::Color::Transparent);
    coordLabel->getRenderer()->setTextColor(tgui::Color::Black);
    gui.add(coordLabel, "CoordLabel");
}

/**
 * @brief Creates and configures the live mode toggle button.
 * 
 * The button switches between ON/OFF states with color feedback (green/red).
 * Positioned near the coordinate label.
 * 
 * @param gui Reference to the TGUI GUI instance.
 */
void MenuBar::setupliveToggle(tgui::Gui& gui) {
    liveToggle = tgui::ToggleButton::create();
    liveToggle->setSize(50, 20);
    liveToggle->setText("Live");
    liveToggle->setPosition("100% - 60", "100% - 20");

    auto renderer = liveToggle->getRenderer();
    renderer->setRoundedBorderRadius(10);
    renderer->setTextColor(tgui::Color::White);
    renderer->setBorderColor(tgui::Color::White);

    // Default OFF (red) colors
    renderer->setBackgroundColor(tgui::Color(120, 0, 0));
    renderer->setBackgroundColorHover(tgui::Color(180, 60, 60));
    renderer->setBackgroundColorDown(tgui::Color(255, 100, 100));

    gui.add(liveToggle);

    liveMode = false;

    // Change toggle button color based on state (green for ON, red for OFF)
    liveToggle->onToggle([this](bool state) {
        liveMode = state;
        auto renderer = liveToggle->getRenderer();

        if (state) {
            // ON state colors (green)
            renderer->setBackgroundColor(tgui::Color(0, 180, 0));
            renderer->setBackgroundColorHover(tgui::Color(80, 220, 80));
            renderer->setBackgroundColorDown(tgui::Color(100, 255, 100));
        } else {
            // OFF state colors (red)
            renderer->setBackgroundColor(tgui::Color(120, 0, 0));
            renderer->setBackgroundColorHover(tgui::Color(180, 60, 60));
            renderer->setBackgroundColorDown(tgui::Color(255, 100, 100));
        }
    });
}

/**
 * @brief Connects external callback functions to the menu items.
 * 
 * Allows external code to respond to menu item selections.
 * 
 * @param onOpen Callback invoked when "Open" is selected.
 * @param onSave Callback invoked when "Save" is selected.
 * @param onSaveImage Callback invoked when "Save2Image" is selected.
 * @param onClose Callback invoked when "Close" is selected.
 * @param onResetView Callback invoked when "ResetView" is selected.
 * @param onClearView Callback invoked when "ClearView" is selected.
 * @param onCreateRobot Callback invoked when "WMR" is selected under "Create object".
 * @param onSimulation Callback invoked when "A*" is selected under "Simulation".
 */
void MenuBar::setCallbacks(
    std::function<void()> onOpen,
    std::function<void()> onSave,
    std::function<void()> onSaveImage,
    std::function<void()> onClose,
    std::function<void()> onResetView,
    std::function<void()> onClearView,
    std::function<void()> onCreateRobot,
    std::function<void()> onSimulation)
{
    menuBar->connectMenuItem("File", "Open", [onOpen]() {
        if (onOpen) onOpen();
    });

    menuBar->connectMenuItem("File", "Save", [onSave]() {
        if (onSave) onSave();
    });

    menuBar->connectMenuItem("File", "Save2Image", [onSaveImage]() {
        if (onSaveImage) onSaveImage();
    });

    menuBar->connectMenuItem("File", "Close", [onClose]() {
        if (onClose) onClose();
    });

    menuBar->connectMenuItem("View", "ResetView", [onResetView]() {
        if (onResetView) onResetView();
    });

    menuBar->connectMenuItem("View", "ClearView", [onClearView]() {
        if (onClearView) onClearView();
    });

    menuBar->connectMenuItem("Create object", "WMR", [onCreateRobot]() {
        if (onCreateRobot) onCreateRobot();
    });

    menuBar->connectMenuItem("Simulation", "A*", [onSimulation]() {
        if (onSimulation) onSimulation();
    });
}

/**
 * @brief Updates the coordinate label text.
 * 
 * @param text New coordinate string to display.
 */
void MenuBar::updateCoordinates(const std::string& text) {
    if (coordLabel)
        coordLabel->setText(text);
}
