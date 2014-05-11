#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QBoxLayout>
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_MainGLWidget = new MainGLWindow();
    setCentralWidget(QWidget::createWindowContainer(m_MainGLWidget));

    m_SubtractSphereButton = new QPushButton("Subtract sphere");
    m_MergeSphereButton = new QPushButton("Add sphere");
    m_CutPlaneButton = new QPushButton("Cutplane");

    m_SubtractSphereButton->setCheckable(true);
    m_SubtractSphereButton->setChecked(true);
    m_SubtractSphereButton->setFlat(true);
    m_MergeSphereButton->setCheckable(true);
    m_MergeSphereButton->setFlat(true);
    m_CutPlaneButton->setCheckable(true);
    m_CutPlaneButton->setFlat(true);

    connect(m_SubtractSphereButton, SIGNAL(released()), m_MainGLWidget, SLOT(setSubtractSphereMode()));
    connect(m_MergeSphereButton, SIGNAL(released()), m_MainGLWidget, SLOT(setMergeSphereMode()));
    connect(m_CutPlaneButton, SIGNAL(released()), m_MainGLWidget, SLOT(setCutPlaneMode()));

    connect(ui->actionLoad_mesh, SIGNAL(triggered()), this, SLOT(openLoadMeshDialog()));

    m_ToolsButtonGroup = new QButtonGroup(ui->toolBar);
    m_ToolsButtonGroup->addButton(m_SubtractSphereButton);
    m_ToolsButtonGroup->addButton(m_MergeSphereButton);
    m_ToolsButtonGroup->addButton(m_CutPlaneButton);

    ui->toolBar->addWidget(m_SubtractSphereButton);
    ui->toolBar->addWidget(m_MergeSphereButton);
    ui->toolBar->addWidget(m_CutPlaneButton);
}

void MainWindow::openLoadMeshDialog()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open File",
                                                    "",
                                                    "Meshes (*.obj)");
    if (fileName != "")
        m_MainGLWidget->loadMesh(fileName);

}

MainWindow::~MainWindow()
{
    delete ui;
}
