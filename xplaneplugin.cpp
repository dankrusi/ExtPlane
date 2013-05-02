#include "xplaneplugin.h"
#include "datarefs/dataref.h"
#include "datarefs/floatdataref.h"
#include "datarefs/floatarraydataref.h"
#include "datarefs/intdataref.h"
#include "datarefs/intarraydataref.h"
#include "datarefs/doubledataref.h"
#include "datarefs/datadataref.h"

XPlanePlugin::XPlanePlugin(QObject *parent) :
    QObject(parent), argc(0), argv(0), app(0), server(0), flightLoopInterval(0.31f) { // Default to 30hz
}

XPlanePlugin::~XPlanePlugin() {
    qDebug() << Q_FUNC_INFO;
}

float XPlanePlugin::flightLoop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop,
                               int inCounter, void *inRefcon) {
    foreach(DataRef *ref, refs)
        ref->updateValue();
    app->processEvents();
    return flightLoopInterval;
}






#include "XPLMNavigation.h"
#include "XPLMGraphics.h"
#include <QList>
#include <QTime>
#include <qmath.h>



bool processed = false;

int GetNavDataCallback(void *inRefcon, void *outValue, int inOffset, int inMaxLength) {

    QString data;

    // Test nav pos local coord
    if(false) {
        XPLMNavRef nav = XPLMFindNavAid(NULL,"KSEA",NULL,NULL,NULL,xplm_Nav_Airport);
        if(nav != XPLM_NAV_NOT_FOUND) {
            float lon,lat;
            XPLMGetNavAidInfo(nav,NULL,&lat,&lon,NULL,NULL,NULL,NULL,NULL,NULL);
            double localX,localY,localZ;
            XPLMWorldToLocal(lat,lon,0,&localX,&localY,&localZ);
            qDebug() << "first nav" << localX << localY;
        }
        double planeX = XPLMGetDatad(XPLMFindDataRef("sim/flightmodel/position/local_x"));
        double planeY = XPLMGetDatad(XPLMFindDataRef("sim/flightmodel/position/local_y"));
        qDebug() << "plane" << planeX << planeY;
    }

    if(!processed) {


        qDebug() << "\n\nGetNavDataCallback\n\n";
        //processed = true;



        // Init
        int count = 0;
        XPLMNavRef nav;
        QTime timer;

        // Get all
        if(false) {
            count = 0;
            timer.restart();
            nav = XPLMGetFirstNavAid();
            while(nav != XPLM_NAV_NOT_FOUND) {
                count++;
                // Get next
                nav = XPLMGetNextNavAid(nav);
                float lon,lat;
                XPLMGetNavAidInfo(nav,NULL,&lon,&lat,NULL,NULL,NULL,NULL,NULL,NULL);
            }
            qDebug() << "total navs: " << count << "ms:" << timer.elapsed();
        }

        // Get all local
        if(true) {
            count = 0;
            timer.restart();

            double planeX = XPLMGetDatad(XPLMFindDataRef("sim/flightmodel/position/local_x"));
            double planeY = XPLMGetDatad(XPLMFindDataRef("sim/flightmodel/position/local_y"));

            nav = XPLMGetFirstNavAid();
            while(nav != XPLM_NAV_NOT_FOUND) {
                // Get next
                nav = XPLMGetNextNavAid(nav);
                float lon,lat;
                char id[32];
                XPLMGetNavAidInfo(nav,NULL,&lat,&lon,NULL,NULL,NULL,id,NULL,NULL);
                double localX,localY,localZ;
                XPLMWorldToLocal(lat,lon,0,&localX,&localY,&localZ);
                double dist = qSqrt( (localX - planeX)*(localX - planeX) + (localY - planeY)*(localY - planeY) );
                if(dist < 200) { // 2km
                    count++;
                    qDebug() << "nav #"<< count << "id" << id;
                    data = QString("nav#%1id%2").arg(count).arg(id);
                }
            }
            qDebug() << "total navs: " << count << "ms:" << timer.elapsed();
        }


        // Get based on grid
        if(false) {
            QList<XPLMNavRef> navs;
            count = 0;
            timer.restart();
            for(int xx = 0; xx < 20; xx++) {
                for(int yy = 0; yy < 20; yy++) {
                    double dlat, dlon, dalt;
                    float flat = (float)dlat;
                    float flon = (float)dlon;
                    XPLMLocalToWorld(xx,yy,0,&dlat,&dlon,&dalt);
                    nav = XPLMFindNavAid(NULL,
                                         NULL,
                                         &flat,    /* float *              inLat Can be NULL */
                                         &flon,    /* float *              inLon Can be NULL */
                                         NULL,
                                         xplm_Nav_Airport);
                    if(nav != XPLM_NAV_NOT_FOUND) {
                        if(!navs.contains(nav)) {
                            navs.append(nav);
                            count++;
                        }
                    }
                }
            }
            qDebug() << "local navs: " << count << "ms:" << timer.elapsed();
        }

        qDebug() << "\n\n";
    }

    if(outValue != NULL) {
        for(int i = 0; i < data.length(); i++) {
            ((char*)outValue)[i] = (char)data.at(i).toAscii();
        }
    }

    return 1000;
}







int XPlanePlugin::pluginStart(char * outName, char * outSig, char *outDesc) {
    qDebug() << Q_FUNC_INFO << "ExtPlane plugin started";
    app = new QCoreApplication(argc, &argv);
    server = new TcpServer(this, this);
    connect(server, SIGNAL(setFlightLoopInterval(float)), this, SLOT(setFlightLoopInterval(float)));
    strcpy(outName, "ExtPlane");
    strcpy(outSig, "org.vranki.extplaneplugin");
    strcpy(outDesc, "Read and write X-Plane datarefs from external programs using TCP socket.");

    //  Create our custom integer dataref
    XPLMDataRef gCounterDataRef = XPLMRegisterDataAccessor("extplane/navdata/test",
                                                 xplmType_Data,                                 // The types we support
                                                 0,                                             // Writable
                                                 NULL, NULL,                                    // Integer accessors
                                                 NULL, NULL,                                    // Float accessors
                                                 NULL, NULL,                                    // Doubles accessors
                                                 NULL, NULL,                                    // Int array accessors
                                                 NULL, NULL,                                    // Float array accessors
                                                 GetNavDataCallback, NULL,                     // Raw data accessors
                                                 NULL, NULL);                                   // Refcons not used

    app->processEvents();
    return 1;
}

DataRef* XPlanePlugin::subscribeRef(QString name) {
    qDebug() << Q_FUNC_INFO << name;
    foreach(DataRef *ref, refs) {
        if(ref->name()==name) {
            qDebug() << Q_FUNC_INFO << "Already subscribed to " << name;
            ref->setSubscribers(ref->subscribers() + 1);
            return ref;
        }
    }

    XPLMDataRef ref = XPLMFindDataRef(name.toLatin1());
    if(ref) {
        XPLMDataTypeID refType = XPLMGetDataRefTypes(ref);
        DataRef *dr = 0;
        if(refType & xplmType_Double) {
            dr = new DoubleDataRef(this, name, ref);
        } else if(refType & xplmType_Float) {
            dr = new FloatDataRef(this, name, ref);
        } else if(refType & xplmType_Int) {
            dr = new IntDataRef(this, name, ref);
        } else if (refType & xplmType_FloatArray) {
            dr = new FloatArrayDataRef(this, name, ref);
        } else if (refType & xplmType_IntArray) {
            dr = new IntArrayDataRef(this, name, ref);
        } else if (refType & xplmType_Data) {
            dr = new DataDataRef(this, name, ref);
        }
        if(dr) {
            dr->setSubscribers(1);
            dr->setWritable(XPLMCanWriteDataRef(ref) != 0);
            qDebug() << Q_FUNC_INFO << "Subscribed to ref " << dr->name() << ", type: " << dr->typeString() << ", writable:" << dr->isWritable();
            refs.append(dr);
            return dr;
        } else {
            qDebug() << Q_FUNC_INFO << "Dataref type " << refType << "not supported";
        }
    } else {
        qDebug() << Q_FUNC_INFO << "Can't find dataref " << name;
    }
    return 0;
}

void XPlanePlugin::unsubscribeRef(DataRef *ref) {
    Q_ASSERT(refs.contains(ref));
    qDebug() << Q_FUNC_INFO << ref->name() << ref->subscribers();
    ref->setSubscribers(ref->subscribers() - 1);
    if(ref->subscribers() == 0) {
        refs.removeOne(ref);
        qDebug() << Q_FUNC_INFO << "Ref " << ref->name() << " not subscribed by anyone - removing.";
        ref->deleteLater();
    }
}

void XPlanePlugin::keyStroke(int keyid) {
    qDebug() << Q_FUNC_INFO << keyid;

    XPLMCommandKeyStroke(keyid);
}

void XPlanePlugin::buttonPress(int buttonid) {
    qDebug() << Q_FUNC_INFO << buttonid;
    XPLMCommandButtonPress(buttonid);
}

void XPlanePlugin::buttonRelease(int buttonid) {
    qDebug() << Q_FUNC_INFO << buttonid;
    XPLMCommandButtonRelease(buttonid);
}

void XPlanePlugin::setFlightLoopInterval(float newInterval) {
    if(newInterval > 0) {
        flightLoopInterval = newInterval;
        qDebug() << Q_FUNC_INFO << "new interval" << flightLoopInterval;
    } else {
        qDebug() << Q_FUNC_INFO << "Invalid interval " << newInterval;
    }
}

void XPlanePlugin::pluginStop() {
    qDebug() << Q_FUNC_INFO;
    app->processEvents();
    delete server;
    server = 0;
    app->quit();
    app->processEvents();
    delete app;
    app = 0;
    qDeleteAll(refs);
    refs.clear();
}

void XPlanePlugin::receiveMessage(XPLMPluginID inFromWho, long inMessage, void *inParam) {
    qDebug() <<  Q_FUNC_INFO << inFromWho << inMessage;
}
