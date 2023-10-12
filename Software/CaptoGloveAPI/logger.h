#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QMutex>
#include <QDate>
#include <QSettings>
#include <QStandardPaths>

#include <QObject>
#include <QVariant>
#include <QtCore>

#include <stdio.h>

#include <proto_impl/captoglove_v1.pb.h>

#define INFO qDebug() << Logger::getTimeStamp() << "[INFO]"
#define DUMP qDebug() << Logger::getTimeStamp() << "[DUMP]"
#define WARN qWarning() << Logger::getTimeStamp() << "[WARN]"
#define ERRO qCritical() << Logger::getTimeStamp() << "[ERRO]"

class Logger : public QObject
{
    Q_OBJECT

    typedef enum{
        DiagInfo=0,
        DiagError,
        DiagTrace,
        DiagDebug,
        DiagWarning,
        DiagProtobuf
    }DiagType;

private:
    explicit Logger(QObject *parent = 0);

public:


    static Logger * instance() {
        static Logger * _instance = 0;
        if ( _instance == 0 ) {
            _instance = new Logger();
        }
        return _instance;
    }

    void loadSettings     (QSettings *Setting);
    void saveSettings     (QSettings *Setting);

    bool start        (void);
    bool stop         (void);

    bool sendMessageToLog   (QString message);

    QString filePath() const;
    void setFilePath(const QString &FilePath);

    QString fileName() const;
    void setFileName(const QString &FileName);

    QString getTimeStamp();


signals:
    void diagSignal     (DiagType Type, QString Text);
    void logSignal      (captoglove_v1::LogMsg log);

public slots:
    void protobufPrintSlot      (google::protobuf::Message *msg);
    void diagSlot               (DiagType Type, QString Text);
    void diagSlot               (int Type, QString Text);

private:
    bool openLogFile        (QString filePath);
    bool closeLogFile       (void);

    QMap<DiagType, bool> m_screenDiag;
    QMap<DiagType, bool> m_logDiag;
    QString m_filePath;
    QString m_fileName;
    QMutex  m_mutex;
    QDate   m_lastLogDate;
    QFile   m_file;
    bool    m_active;

};


#endif // LOGGER_H
