#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QBoxLayout>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_MainGLWidget = new MainGLWindow();
    setCentralWidget(QWidget::createWindowContainer(m_MainGLWidget));
}

MainWindow::~MainWindow()
{
    delete ui;
}
