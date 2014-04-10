#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QBoxLayout>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_MainGLWidget = new MainGLWidget(this);
    setCentralWidget(m_MainGLWidget);
}

MainWindow::~MainWindow()
{
    delete ui;
}
