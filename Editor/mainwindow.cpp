#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "pointlight.h"
#include "directionallight.h"
#include "ecsmanager.h"
#include "renderwindow.h"
#include "coreengine.h"

#include <QSurfaceFormat>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QTextEdit>
#include <QDebug>


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    init();
}

MainWindow::~MainWindow()
{
    delete mRenderWindow;
    delete ui;
}

void MainWindow::init()
{
    //Setting up the OpenGL 4.1 surface:
    QSurfaceFormat format;
    format.setVersion(4, 1);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setSamples(4);    //Set the number of samples used for multisampling
    format.setOption(QSurfaceFormat::DebugContext); //Should activate OpenGL debug Context used in RenderWindow::startOpenGLDebugger()
    format.setDepthBufferSize(24);      //The example rendering will need a depth buffer.
    qDebug() << "Requesting surface format" << format;

    //Making the renderwindow with this surface - testing if it went well:
    mRenderWindow = new RenderWindow(format, this);
    if (!mRenderWindow->context())
    {
        qDebug() << (tr("Failed to create context"));
        mRenderWindowContainer = new QWidget(this);
        addRenderWindow();
        return;
    }

    connect(mRenderWindow, &RenderWindow::ready, this, &MainWindow::renderWindowReady);
    connect(mRenderWindow, &RenderWindow::error, this, &MainWindow::renderWindowError);

    //Adds the renderwindow to the visual program - inside the
    //RenderWindowContainer that we can see in the form-designer.
    mRenderWindowContainer = QWidget::createWindowContainer(mRenderWindow);
    addRenderWindow();

    mRenderWindowContainer->setFocus(); //Sets the keyboard input focus to this window when program starts
}

void MainWindow::renderWindowReady()
{
    //Testing if the context is valid
    QOpenGLContext *context = QOpenGLContext::currentContext();
    Q_ASSERT(context);

    mCoreEngine = new CoreEngine(mRenderWindow, this);
    mRenderWindow->setCoreEngine(mCoreEngine);

    setLightColor();
    setColorPreview();
    updateWorldOutliner();
}

void MainWindow::renderWindowError(const QString &msg)
{
    //Generic error writing. Could easly be removed and handeled in other ways
    qDebug() << (tr("An error has occurred:\n%1").arg(msg));
}

void MainWindow::addRenderWindow()
{
    ui->renderLayout->addWidget(mRenderWindowContainer);
}

void MainWindow::updateWorldOutliner()
{
    CoreEngine* coreEngine = mCoreEngine;
    ECSManager* ecsManager = coreEngine->mECSManager;

    std::list<Entity*> *entities = &ecsManager->mEntities;
    for (auto tempEntity: *entities)
    {
        ui->lwHierarchy->addItem(QString::fromStdString(tempEntity->getObjectName()));
    }
}

void MainWindow::on_actionWireframe_toggled(bool arg1)
{
    mRenderWindow->setWireframe(arg1);
}

void MainWindow::on_actionAxis_toggled(bool arg1)
{
    mRenderWindow->setShowAxis(!arg1);  //Starts with true
}

void MainWindow::on_lightIntensitySlider_sliderMoved(int position)
{
    mCoreEngine->mECSManager->getLight()->lightIntensity = position/255.f;
    setColorPreview();
}

void MainWindow::on_lightAmbientRedSlider_sliderMoved(int position)
{
    mCoreEngine->mECSManager->getLight()->mLightColor.setX(position/255.f);
    setColorPreview();
}

void MainWindow::on_lightAmbientGreenSlider_sliderMoved(int position)
{
    mCoreEngine->mECSManager->getLight()->mLightColor.setY(position/255.f);
    setColorPreview();
}

void MainWindow::on_lightAmbientBlueSlider_sliderMoved(int position)
{
    mCoreEngine->mECSManager->getLight()->mLightColor.setZ(position/255.f);
    setColorPreview();
}

void MainWindow::setColorPreview()
{
    GLint red = ui->lightAmbientRedSlider->value();
    GLint green = ui->lightAmbientGreenSlider->value();
    GLint blue = ui->lightAmbientBlueSlider->value();
    GLint power = ui->lightIntensitySlider->value();

    //Background-color: rgb(255, 0, 4)
    QString result = QString("background-color: rgb(") + QString::number(red * power/255) + QString(", ") +
            QString::number(green * power/255) + QString(", ") + QString::number(blue * power/255) + QString(")");

    ui->lightAmbientPreview->setStyleSheet(result);
}

void MainWindow::setLightColor()
{
    ECSManager* manager = mCoreEngine->mECSManager;
    DirectionalLight* light = manager->getLight();
    ui->lightAmbientRedSlider->setValue(static_cast<GLint>(255*light->mLightColor.getX()));
    ui->lightAmbientGreenSlider->setValue(static_cast<GLint>(255*light->mLightColor.getY()));
    ui->lightAmbientBlueSlider->setValue(static_cast<GLint>(255*light->mLightColor.getZ()));
    ui->lightIntensitySlider->setValue(static_cast<GLint>(255*light->lightIntensity));
}

void MainWindow::setLocationValues(gsl::Vec3 locationValues)
{
    ui->LocationX->setValue(static_cast<GLdouble>(locationValues.getX()));
    ui->LocationY->setValue(static_cast<GLdouble>(locationValues.getY()));
    ui->LocationZ->setValue(static_cast<GLdouble>(locationValues.getZ()));
}

void MainWindow::setRotationValues(gsl::Vec3 rotationValues)
{
    ui->RotationX->setValue(static_cast<GLdouble>(rotationValues.getX()));
    ui->RotationY->setValue(static_cast<GLdouble>(rotationValues.getY()));
    ui->RotationZ->setValue(static_cast<GLdouble>(rotationValues.getZ()));
}

void MainWindow::setScaleValues(gsl::Vec3 scaleValues)
{
    ui->ScaleX->setValue(static_cast<GLdouble>(scaleValues.getX()));
    ui->ScaleY->setValue(static_cast<GLdouble>(scaleValues.getY()));
    ui->ScaleZ->setValue(static_cast<GLdouble>(scaleValues.getZ()));
}

void MainWindow::setSelectedEntity(Entity setSelected)
{
    mSelectedEntity = setSelected;
    setLocationValues(setSelected.getTransform().getPosition());
    setRotationValues(setSelected.getTransform().getRotation().toEulerAngles());
    setScaleValues(setSelected.getTransform().getScale());
}

Entity MainWindow::getSelectedEntity()
{
    return mSelectedEntity;
}

void MainWindow::on_lwHierarchy_itemClicked(QListWidgetItem *item)
{
    GLint itemRow = ui->lwHierarchy->row(item);
    GLint counter{0};
    isSelectedEntity = true;

    std::list<Entity*> sceneObjects = mCoreEngine->mECSManager->mEntities;
    for (auto entities : sceneObjects)
    {
        if (counter == itemRow)
        {
            setSelectedEntity(*entities);
            break;
        }
        else
            counter++;
    }
}

void MainWindow::on_LocationX_valueChanged(double arg1)
{
    if (isSelectedEntity)
    {
        gsl::Vec3 position(static_cast<GLfloat>(arg1),
                      mSelectedEntity.getTransform().getPosition().getY(),
                      mSelectedEntity.getTransform().getPosition().getZ());

        mSelectedEntity.getTransform().setPosition(position);
    }
}

void MainWindow::on_LocationY_valueChanged(double arg1)
{
    gsl::Vec3 position(mSelectedEntity.getTransform().getPosition().getX(),
                  static_cast<GLfloat>(arg1),
                  mSelectedEntity.getTransform().getPosition().getZ());

    mSelectedEntity.getTransform().setPosition(position);
}

void MainWindow::on_LocationZ_valueChanged(double arg1)
{
    gsl::Vec3 position(mSelectedEntity.getTransform().getPosition().getX(),
                  mSelectedEntity.getTransform().getPosition().getY(),
                  static_cast<GLfloat>(arg1));

    mSelectedEntity.getTransform().setPosition(position);
}

void MainWindow::on_RotationX_valueChanged(double arg1)
{
    while (arg1 > 360.0)
    {
        arg1 -= 360.0;
        ui->RotationX->setValue(arg1);
    }
    while (arg1 < -360.0)
    {
        arg1 += 360.0;
        ui->RotationX->setValue(arg1);
    }

    gsl::Vec3 rotation(static_cast<GLfloat>(arg1),
                  mSelectedEntity.getTransform().getRotation().toEulerAngles().getY(),
                  mSelectedEntity.getTransform().getRotation().toEulerAngles().getZ());

    mSelectedEntity.getTransform().setRotation(rotation);
}

void MainWindow::on_RotationY_valueChanged(double arg1)
{
    while (arg1 > 360.0)
    {
        arg1 -= 360.0;
        ui->RotationY->setValue(arg1);
    }
    while (arg1 < -360.0)
    {
        arg1 += 360.0;
        ui->RotationY->setValue(arg1);
    }

    gsl::Vec3 rotation(mSelectedEntity.getTransform().getRotation().toEulerAngles().getX(),
                  static_cast<GLfloat>(arg1),
                  mSelectedEntity.getTransform().getRotation().toEulerAngles().getZ());

    mSelectedEntity.getTransform().setRotation(rotation);
}

void MainWindow::on_RotationZ_valueChanged(double arg1)
{
    while (arg1 > 360.0)
    {
        arg1 -= 360.0;
        ui->RotationZ->setValue(arg1);
    }
    while (arg1 < -360.0)
    {
        arg1 += 360.0;
        ui->RotationZ->setValue(arg1);
    }

    gsl::Vec3 rotation(mSelectedEntity.getTransform().getRotation().toEulerAngles().getX(),
                  mSelectedEntity.getTransform().getRotation().toEulerAngles().getY(),
                  static_cast<GLfloat>(arg1));

    mSelectedEntity.getTransform().setRotation(rotation);
}

void MainWindow::on_ScaleX_valueChanged(double arg1)
{
    gsl::Vec3 scale(static_cast<GLfloat>(arg1),
               mSelectedEntity.getTransform().getScale().getY(),
               mSelectedEntity.getTransform().getScale().getZ());

    mSelectedEntity.getTransform().setScale(scale);
}

void MainWindow::on_ScaleY_valueChanged(double arg1)
{
    gsl::Vec3 scale(mSelectedEntity.getTransform().getScale().getX(),
               static_cast<GLfloat>(arg1),
               mSelectedEntity.getTransform().getScale().getZ());

    mSelectedEntity.getTransform().setScale(scale);
}

void MainWindow::on_ScaleZ_valueChanged(double arg1)
{
    gsl::Vec3 scale(mSelectedEntity.getTransform().getScale().getX(),
               mSelectedEntity.getTransform().getScale().getY(),
               static_cast<GLfloat>(arg1));

    mSelectedEntity.getTransform().setScale(scale);
}
