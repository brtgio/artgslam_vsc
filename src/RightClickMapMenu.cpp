#include "artgslam_vsc/RightClickMapMenu.hpp"
#include <ros/package.h>
#include <iostream>

/**
 * @brief Constructs the RightClickMapMenu.
 * 
 * Initializes references to GUI, GridMap, and LiveMap, then sets up the menu widgets
 * and connects button signals.
 * 
 * @param guiRef Reference to the TGUI GUI instance.
 * @param mapRef Reference to the GridMap instance.
 * @param livemapRef Reference to the LiveMap instance.
 */
RightClickMapMenu::RightClickMapMenu(tgui::Gui& guiRef, GridMap& mapRef, LiveMap& livemapRef)
    : gui(guiRef), gridmap(mapRef), livemap(livemapRef)
{
    std::cout << "📦 RightClickMapMenu: loading..." << std::endl;
    setupWidgets();    /**< Load and configure GUI widgets */
    connectSignals();  /**< Connect button event handlers */
}

/**
 * @brief Loads GUI widgets from external TGUI form and applies styles.
 * 
 * Retrieves the ROS package path to locate the GUI form file, loads widgets into
 * a container, then retrieves and styles the individual buttons. Hides the panel by default.
 */
void RightClickMapMenu::setupWidgets()
{
    std::string package_path = ros::package::getPath("artgslam_vsc");
    std::string formPath = package_path + "/assets/forms/Right_Click_Menu.txt";

    try {
        container = tgui::Group::create();
        container->loadWidgetsFromFile(formPath);
        gui.add(container);

        panel = container->get<tgui::ChildWindow>("ChildWindow1");
        if (!panel) {
            std::cerr << "❌ 'ChildWindow1' widget not found in form." << std::endl;
            return;
        }

        auto layout = panel->get<tgui::VerticalLayout>("VerticalLayout1");
        if (!layout) {
            std::cerr << "❌ 'VerticalLayout1' widget not found in panel." << std::endl;
            return;
        }

        start     = layout->get<tgui::Button>("start");
        goal      = layout->get<tgui::Button>("goal");
        clear     = layout->get<tgui::Button>("Clear");
        clearView = layout->get<tgui::Button>("ClearView");

        if (!start || !goal || !clear || !clearView) {
            std::cerr << "⚠️ One or more buttons not found in layout." << std::endl;
        } else {
            auto applyHoverStyle = [](tgui::Button::Ptr btn) {
                auto renderer = btn->getRenderer();
                renderer->setBackgroundColor(sf::Color::Transparent);
                renderer->setBackgroundColorHover(sf::Color(0, 120, 215)); // Windows 10 blue
                renderer->setTextColor(sf::Color::Black);
                renderer->setTextColorHover(sf::Color::White);
                renderer->setBorderColor(sf::Color::Transparent);
                renderer->setBorderColorHover(sf::Color(0, 120, 215));
                renderer->setBorders({1, 1, 1, 1}); // 1-pixel border
            };

            applyHoverStyle(start);
            applyHoverStyle(goal);
            applyHoverStyle(clear);
            applyHoverStyle(clearView);
        }

        panel->setVisible(false); /**< Hide panel initially */

    } catch (const tgui::Exception& e) {
        std::cerr << "❌ Failed to load TGUI form: " << e.what() << std::endl;
    }
}

/**
 * @brief Shows the right-click menu at specified screen coordinates with a selected grid index.
 * 
 * @param x Screen x position.
 * @param y Screen y position.
 * @param gridIndex Grid cell index selected.
 */
void RightClickMapMenu::show(float x, float y, const sf::Vector2i& gridIndex)
{
    if (!panel) return;

    gridIndex_copy = gridIndex;

    std::cout << "🟢 Selected index: (" << gridIndex.x << ", " << gridIndex.y << ")" << std::endl;

    panel->setPosition(x, y);
    panel->setVisible(true);
    visible = true;
}

/**
 * @brief Hides the right-click menu.
 */
void RightClickMapMenu::hide()
{
    if (panel) {
        panel->setVisible(false);
        visible = false;
    }
}

/**
 * @brief Explicitly sets the visibility of the menu.
 * 
 * @param show True to show the menu, false to hide.
 */
void RightClickMapMenu::setVisible(bool show)
{
    if (panel) {
        panel->setVisible(show);
        visible = show;
    }
}

/**
 * @brief Returns whether the menu is currently visible.
 * 
 * @return true if visible, false otherwise.
 */
bool RightClickMapMenu::isVisible() const
{
    return visible;
}

/**
 * @brief Determines if a given point lies inside the menu boundaries.
 * 
 * @param point Point in screen coordinates to test.
 * @return true if point is inside menu bounds, false otherwise.
 */
bool RightClickMapMenu::containsPoint(const sf::Vector2f& point) const
{
    if (!panel) return false;

    sf::Vector2f pos = panel->getAbsolutePosition();
    sf::Vector2f size = panel->getSize();
    sf::FloatRect bounds(pos.x, pos.y, size.x, size.y);

    return bounds.contains(point);
}

/**
 * @brief Connects button press signals to their respective logic functions.
 * 
 * Handles setting start/goal points, clearing points, and other button actions.
 */
void RightClickMapMenu::connectSignals()
{
    if (start) {
        start->onPress([this]() {
            if (!isStartActive) {
                if (livemap.getIsActive()) {
                    livemap.setStart(gridIndex_copy.x, gridIndex_copy.y);
                    std::cout << "📍 Start set on LiveMap." << std::endl;
                } else {
                    gridmap.setStart(gridIndex_copy.x, gridIndex_copy.y);
                    std::cout << "📍 Start set on GridMap." << std::endl;
                }
                startIndex = gridIndex_copy;
                isStartActive = true;
            } else {
                std::cout << "⚠️ Start is already active. Clear it first." << std::endl;
            }
            hide();
        });
    }

    if (goal) {
        goal->onPress([this]() {
            if (!isGoalActive) {
                if (livemap.getIsActive()) {
                    livemap.setGoal(gridIndex_copy.x, gridIndex_copy.y);
                    std::cout << "🎯 Goal set on LiveMap." << std::endl;
                } else {
                    gridmap.setGoal(gridIndex_copy.x, gridIndex_copy.y);
                    std::cout << "🎯 Goal set on GridMap." << std::endl;
                }
                goalIndex = gridIndex_copy;
                isGoalActive = true;
            } else {
                std::cout << "⚠️ Goal is already active. Clear it first." << std::endl;
            }
            hide();
        });
    }

    if (clear) {
        clear->onPress([this]() {
            if (isStartActive) {
                gridmap.clearSetPoints(startIndex);
                isStartActive = false;
            }
            if (isGoalActive) {
                gridmap.clearSetPoints(goalIndex);
                isGoalActive = false;
            }
            std::cout << "🧹 Start and Goal cleared." << std::endl;
            hide();
        });
    }

    if (clearView) {
        clearView->onPress([this]() {
            std::cout << "🧼 Clear View button pressed (no action assigned)." << std::endl;
            hide();
        });
    }
}
