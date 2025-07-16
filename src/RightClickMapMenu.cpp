#include "artgslam_vsc/RightClickMapMenu.hpp"
#include <ros/package.h>
#include <iostream>

RightClickMapMenu::RightClickMapMenu(tgui::Gui& guiRef,GridMap& mapRef,LiveMap& livemapRef )
    : gui(guiRef), gridmap(mapRef),livemap(livemapRef)
{
    std::cout << "📦 RightClickMapMenu: loading..." << std::endl;
    setupWidgets();
    connectSignals();
}

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
            std::cerr << "❌ No se encontró 'ChildWindow1' en el form.\n";
            return;
        }

        auto layout = panel->get<tgui::VerticalLayout>("VerticalLayout1");
        if (!layout) {
            std::cerr << "❌ No se encontró 'VerticalLayout1' dentro del panel.\n";
            return;
        }

        start     = layout->get<tgui::Button>("start");
        goal      = layout->get<tgui::Button>("goal");
        clear     = layout->get<tgui::Button>("Clear");
        clearView = layout->get<tgui::Button>("ClearView");

        if (!start || !goal || !clear || !clearView) {
            std::cerr << "⚠️ Uno o más botones no fueron encontrados dentro del layout.\n";
        } else {
            // Aplicar estilo para que el fondo sea azul en hover y borde visible
            auto applyHoverStyle = [](tgui::Button::Ptr btn) {
                btn->getRenderer()->setBackgroundColor(sf::Color::Transparent);
                btn->getRenderer()->setBackgroundColorHover(sf::Color(0, 120, 215));  // Azul Windows 10
                btn->getRenderer()->setTextColor(sf::Color::Black);
                btn->getRenderer()->setTextColorHover(sf::Color::White);
                btn->getRenderer()->setBorderColor(sf::Color::Transparent);
                btn->getRenderer()->setBorderColorHover(sf::Color(0, 120, 215));
                btn->getRenderer()->setBorders({1, 1, 1, 1});  // Borde de 1 píxel en todos lados
            };

            applyHoverStyle(start);
            applyHoverStyle(goal);
            applyHoverStyle(clear);
            applyHoverStyle(clearView);
        }

        panel->setVisible(false);

    } catch (const tgui::Exception& e) {
        std::cerr << "❌ Error al cargar el formulario TGUI: " << e.what() << std::endl;
    }
}

void RightClickMapMenu::show(float x, float y,  const sf::Vector2i& gridIndex)
{
    if (!panel) return;

   
    gridIndex_copy = gridIndex;

    std::cout << "🟢 Selected index: (" << gridIndex.x << ", " << gridIndex.y << ")\n";

    panel->setPosition(x, y);
    panel->setVisible(true);
    visible = true;
}

void RightClickMapMenu::hide()
{
    if (panel) {
        panel->setVisible(false);
        visible = false;
    }
}

void RightClickMapMenu::setVisible(bool show)
{
    if (panel) {
        panel->setVisible(show);
        visible = show;
    }
}

bool RightClickMapMenu::isVisible() const
{
    return visible;
}

bool RightClickMapMenu::containsPoint(const sf::Vector2f& point) const
{
    if (!panel)
        return false;

    sf::Vector2f pos = panel->getAbsolutePosition();
    sf::Vector2f size = panel->getSize();
    sf::FloatRect bounds(pos.x, pos.y, size.x, size.y);

    return bounds.contains(point);
}


void RightClickMapMenu::connectSignals()
{
    if (start) {
        start->onPress([this]() {
            if (!isStartActive) {
                if (livemap.getIsActive()) {
                    livemap.setStart(gridIndex_copy.x, gridIndex_copy.y);
                    std::cout<<"los indices son x:"<<gridIndex_copy.x<<" y:"<<std::endl;
                    std::cout << "📍 Start set on Livemap.\n";
                } else {
                    gridmap.setStart(gridIndex_copy.x, gridIndex_copy.y);
                    std::cout << "📍 Start set on GridMap.\n";
                }
                startIndex = gridIndex_copy;
                isStartActive = true;
            } else {
                std::cout << "⚠️ Start already active. Clear it first.\n";
            }
            hide();
        });
    }

    if (goal) {
        goal->onPress([this]() {
            if (!isGoalActive) {
                if (livemap.getIsActive()) {
                    livemap.setGoal(gridIndex_copy.x, gridIndex_copy.y);

                    std::cout << "🎯 Goal set on Livemap.\n";
                } else {
                    gridmap.setGoal(gridIndex_copy.x, gridIndex_copy.y);

                    std::cout << "🎯 Goal set on GridMap.\n";
                }
                std::cout<<"los indices son x:"<<gridIndex_copy.x<<" y:"<<std::endl;
                goalIndex = gridIndex_copy;
                isGoalActive = true;
            } else {
                std::cout << "⚠️ Goal already active. Clear it first.\n";
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
            std::cout << "🧹 Start & Goal cleared.\n";
            hide();
        });
    }

    if (clearView) {
        clearView->onPress([this]() {
            std::cout << "🧼 Clear View pressed (no action assigned).\n";
            hide();
        });
    }
}

