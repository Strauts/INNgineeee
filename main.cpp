#include "mainwindow.h"
#include "constants.h"
#include <QApplication>
#include <QSplashScreen>
#include <iostream>


int main(int argc, char *argv[])
{
    //Forces the usage of desktop OpenGL
    //Attribute must be set before Q(Gui)Application is constructed:
    QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);

    QApplication a(argc, argv);

    QSplashScreen *mSplash = new QSplashScreen;
    mSplash->setPixmap(QPixmap(":/icons/INNgine_splash.png")); // splash picture
    mSplash->show();

    MainWindow w;
    w.show();

    int returnValue{0};
    try
    {
        mSplash->close();
        returnValue = a.exec();
    }
    catch (...) // catch-all handler
    {
        std::cout << "We caught an exception of an undetermined type\n";
    }

    return returnValue;
}
