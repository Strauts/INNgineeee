#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "filebrowser.h"
#include "vec3.h"
#include "entity.h"
#include <QMainWindow>
#include <QWidget>
#include <QListWidget>

class QSurface;
class QTextEdit;
class RenderWindow;
class CoreEngine;

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        explicit MainWindow(QWidget *parent = nullptr);
        ~MainWindow();

    private slots:
        void init();
        void renderWindowReady();
        void renderWindowError(const QString &msg);

        void on_actionWireframe_toggled(bool arg1);
        void on_lightIntensitySlider_sliderMoved(int position);
        void on_lightAmbientRedSlider_sliderMoved(int position);
        void on_lightAmbientGreenSlider_sliderMoved(int position);
        void on_lightAmbientBlueSlider_sliderMoved(int position);

        void on_actionAxis_toggled(bool arg1);

        void on_lwHierarchy_itemClicked(QListWidgetItem *item);

        void on_LocationX_valueChanged(double arg1);
        void on_LocationY_valueChanged(double arg1);
        void on_LocationZ_valueChanged(double arg1);

        void on_RotationX_valueChanged(double arg1);
        void on_RotationY_valueChanged(double arg1);
        void on_RotationZ_valueChanged(double arg1);

        void on_ScaleX_valueChanged(double arg1);
        void on_ScaleY_valueChanged(double arg1);
        void on_ScaleZ_valueChanged(double arg1);

    private:
        void addRenderWindow();
        void updateWorldOutliner();

        void setColorPreview();
        void setLightColor();

        void setLocationValues(gsl::Vec3 locationValues);
        void setRotationValues(gsl::Vec3 rotationValues);
        void setScaleValues(gsl::Vec3 scaleValues);

        void setSelectedEntity(Entity setSelected);
        Entity getSelectedEntity();

        Ui::MainWindow *ui{nullptr};
        QWidget *mRenderWindowContainer{nullptr};
        RenderWindow *mRenderWindow{nullptr};
        CoreEngine *mCoreEngine{nullptr};
        FileBrowser* browser{nullptr};

        Entity mSelectedEntity;
        bool isSelectedEntity{false};
};

#endif // MAINWINDOW_H
