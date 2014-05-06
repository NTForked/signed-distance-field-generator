#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QButtonGroup>
#include <QPushButton>
#include "MainGLWindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void openLoadMeshDialog();

private:
    Ui::MainWindow *ui;
    MainGLWindow* m_MainGLWidget;

    QButtonGroup* m_ToolsButtonGroup;
    QPushButton* m_SubtractSphereButton;
    QPushButton* m_MergeSphereButton;
    QPushButton* m_CutPlaneButton;
};

#endif // MAINWINDOW_H
