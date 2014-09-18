#include "StdAfx.h"

#include <QApplication>

#include "pclstudio.h"

int main(int argc, char **argv)
{
    QApplication *app = new QApplication(argc, argv);

    PCLStudio *pclstudio = new PCLStudio();
    pclstudio->show();

    return app->exec();
}


