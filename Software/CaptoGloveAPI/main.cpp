#include <QCoreApplication>

#include <captogloveapi.h>


int main(int argc, char *argv[]){
    QCoreApplication a(argc, argv);

    CaptoGloveAPI *ctrl = new CaptoGloveAPI(NULL,"");
    ctrl->run();

    return a.exec();
}
