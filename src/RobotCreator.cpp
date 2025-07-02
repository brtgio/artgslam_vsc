#include "artgslam_vsc/RobotCreator.hpp"
#include <ros/package.h>
#include <iostream>

RobotCreator::RobotCreator()
: window(sf::VideoMode(400, 250), "Robot Creator"),
  gui(window)
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

void RobotCreator::run(){

     while (isRunning())
    {
        processEvents();
        update();
        render();
    }
}

void RobotCreator::processEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        gui.handleEvent(event);

        if (event.type == sf::Event::Closed)
            window.close();
    }
}

void RobotCreator::update() {
    // Aquí podrías poner lógica si fuera necesaria
}

void RobotCreator::render() {
    window.clear(sf::Color::White);
    gui.draw();
    window.display();
}

void RobotCreator::setupWidgets()
{
    withBox = gui.get<tgui::EditBox>("withBox");
    heightBox = gui.get<tgui::EditBox>("heightBox");
    colorBox = gui.get<tgui::EditBox>("colorBox");

    resetButton = gui.get<tgui::Button>("Button1");
    createButton = gui.get<tgui::Button>("Button2");

    // Validar floats con signo y decimal
    withBox->setInputValidator("^-?\\d*\\.?\\d*$");
    heightBox->setInputValidator("^-?\\d*\\.?\\d*$");
}


void RobotCreator::setupCallbacks()
{
    resetButton->onPress([this]() {
        withBox->setText("");
        heightBox->setText("");
        colorBox->setText("");
    });

    createButton->onPress([this]() {
        std::string widthStr = withBox->getText().toStdString();
        std::string heightStr = heightBox->getText().toStdString();
        std::string colorStr = colorBox->getText().toStdString();

        bool valid = true;

       
        if (!isValidHexColor(colorStr)) {
            colorBox->setText("");
            valid = false;
        }

        if (valid) {
            float width = std::stof(widthStr);
            float height = std::stof(heightStr);
            sf::Color color = hexToColor(colorStr);

            std::cout << "Robot creado con:" << std::endl;
            std::cout << "Width: " << width << ", Height: " << height << std::endl;
            std::cout << "Color RGB: (" << (int)color.r << ", " << (int)color.g << ", " << (int)color.b << ")" << std::endl;

            window.close(); // Cierra la ventana si quieres
        }
    });
}

bool RobotCreator::isRunning() const
{
    return window.isOpen();
}

bool RobotCreator::isValidFloat(const std::string &str)
{
        if (str.empty()) return false;
        std::regex floatRegex(R"(^-?\d*\.?\d+$)");
        return std::regex_match(str, floatRegex);
}

bool RobotCreator::isValidHexColor(const std::string &str)
{
    std::regex hexRegexShort(R"(#([A-Fa-f0-9]{3}))");
    std::regex hexRegexLong(R"(#([A-Fa-f0-9]{6}))");
    return std::regex_match(str, hexRegexShort) || std::regex_match(str, hexRegexLong);
}

sf::Color RobotCreator::hexToColor(const std::string &hex)
{
     std::string h = hex;
    if (h[0] == '#') h = h.substr(1);

    if (h.length() == 3) {
        // Expandir formato #RGB a #RRGGBB
        h = {h[0], h[0], h[1], h[1], h[2], h[2]};
    }

    unsigned int r = std::stoul(h.substr(0, 2), nullptr, 16);
    unsigned int g = std::stoul(h.substr(2, 2), nullptr, 16);
    unsigned int b = std::stoul(h.substr(4, 2), nullptr, 16);

    return sf::Color(r, g, b);
}
