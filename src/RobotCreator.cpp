#include "artgslam_vsc/RobotCreator.hpp"
#include <ros/package.h>
#include <iostream>
#include <regex>

RobotCreator::RobotCreator(UnicicleWmr& wmrRef)
: windowRobotCreator(sf::VideoMode(400, 250), "Robot Creator"),
  gui(windowRobotCreator), wmr(wmrRef)
{
    std::string package_path = ros::package::getPath("artgslam_vsc");
    std::string formPath = package_path + "/assets/forms/createRobot.txt";

    try {
        gui.loadWidgetsFromFile(formPath);
    } catch (const tgui::Exception& e) {
        std::cerr << "Error loading GUI form: " << e.what() << std::endl;
    }

    setupWidgets();
    setupCallbacks();
}

void RobotCreator::run()
{
    while (isRunning())
    {
        processEvents();
        update();
        render();
    }
}

void RobotCreator::processEvents()
{
    sf::Event event;
    while (windowRobotCreator.pollEvent(event))
    {
        gui.handleEvent(event);

        if (event.type == sf::Event::Closed)
            windowRobotCreator.close();
    }
}

void RobotCreator::update()
{
    // Lógica si es necesaria
}

void RobotCreator::render()
{
    windowRobotCreator.clear(sf::Color::White);
    gui.draw();
    windowRobotCreator.display();
}

void RobotCreator::setupWidgets()
{
    widthBox = gui.get<tgui::EditBox>("widthBox");
    heightBox = gui.get<tgui::EditBox>("heightBox");
    colorBox = gui.get<tgui::EditBox>("colorBox");

    resetButton = gui.get<tgui::Button>("Button1");
    createButton = gui.get<tgui::Button>("Button2");

    // Verificación de carga de widgets
    if (!widthBox || !heightBox || !colorBox || !resetButton || !createButton)
    {
        std::cerr << "[ERROR] Algún widget no se encontró en el formulario.\n";
        return;
    }

    // Validadores de números flotantes (positivos o negativos)
    widthBox->setInputValidator(R"(^-?\d*\.?\d+$)");
    heightBox->setInputValidator(R"(^-?\d*\.?\d+$)");
}

void RobotCreator::setupCallbacks()
{
    if (!resetButton || !createButton) return;

    resetButton->onPress([this]() {
        widthBox->setText("");
        heightBox->setText("");
        colorBox->setText("");
    });

    createButton->onPress([this]() {
        std::string widthStr = widthBox->getText().toStdString();
        std::string heightStr = heightBox->getText().toStdString();
        std::string colorStr = colorBox->getText().toStdString();

        bool valid = true;

        if (!isValidFloat(widthStr)) {
            widthBox->setText("");
            valid = false;
        }

        if (!isValidFloat(heightStr)) {
            heightBox->setText("");
            valid = false;
        }

        if (!isValidHexColor(colorStr)) {
            colorBox->setText("");
            valid = false;
        }

        if (!valid) {
            std::cout << "Error: Datos inválidos. Verifica los campos.\n";
            return;
        }

        // Conversión segura después de validar
        float width = std::stof(widthStr);
        float height = std::stof(heightStr);
        sf::Color color = hexToColor(colorStr);

        // Aplicar parámetros al robot
        wmr.setDimentions(width, height);
        wmr.setColor(color);

        std::cout << "Robot creado:\n";
        std::cout << "Width: " << width << ", Height: " << height << "\n";
        std::cout << "Color RGB: (" 
                  << static_cast<int>(color.r) << ", "
                  << static_cast<int>(color.g) << ", "
                  << static_cast<int>(color.b) << ")\n";

        windowRobotCreator.close();
    });
}

bool RobotCreator::isRunning() const
{
    return windowRobotCreator.isOpen();
}

bool RobotCreator::isValidFloat(const std::string& str)
{
    if (str.empty()) return false;
    std::regex floatRegex(R"(^-?\d+(\.\d+)?$)");
    return std::regex_match(str, floatRegex);
}

bool RobotCreator::isValidHexColor(const std::string& str)
{
    std::regex hexShort(R"(#([A-Fa-f0-9]{3}))");
    std::regex hexLong(R"(#([A-Fa-f0-9]{6}))");
    return std::regex_match(str, hexShort) || std::regex_match(str, hexLong);
}

sf::Color RobotCreator::hexToColor(const std::string& hex)
{
    std::string h = hex;
    if (h.empty()) return sf::Color::White;
    if (h[0] == '#') h = h.substr(1);

    if (h.length() == 3)
    {
        h = {h[0], h[0], h[1], h[1], h[2], h[2]};
    }

    unsigned int r = std::stoul(h.substr(0, 2), nullptr, 16);
    unsigned int g = std::stoul(h.substr(2, 2), nullptr, 16);
    unsigned int b = std::stoul(h.substr(4, 2), nullptr, 16);

    return sf::Color(r, g, b);
}
