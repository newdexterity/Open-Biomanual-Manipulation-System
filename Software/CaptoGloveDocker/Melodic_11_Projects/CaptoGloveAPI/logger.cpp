#include "logger.h"

Logger::Logger(QObject *parent) : QObject(parent)
{
    m_active=false;

    m_screenDiag[DiagInfo]      = true;
    m_screenDiag[DiagError]     = true;
    m_screenDiag[DiagTrace]     = true;
    m_screenDiag[DiagDebug]     = true;
    m_screenDiag[DiagWarning]   = true;
    m_screenDiag[DiagProtobuf]  = true;

    m_logDiag[DiagInfo]         = false;
    m_logDiag[DiagError]        = false;
    m_logDiag[DiagTrace]        = false;
    m_logDiag[DiagDebug]        = true;
    m_logDiag[DiagWarning]      = false;
    m_logDiag[DiagProtobuf]     = false;

    m_fileName = "Log";
    m_filePath = QStandardPaths::writableLocation(QStandardPaths::TempLocation);

}

void Logger::loadSettings     (QSettings *Setting)
{
    m_screenDiag[DiagInfo]      = Setting->value("ScreenInfo"   ,m_screenDiag[DiagInfo]).toBool();
    m_screenDiag[DiagError]     = Setting->value("ScreenError"  ,m_screenDiag[DiagError]).toBool();
    m_screenDiag[DiagTrace]     = Setting->value("ScreenTrace"  ,m_screenDiag[DiagTrace]).toBool();
    m_screenDiag[DiagDebug]     = Setting->value("ScreenDebug"  ,m_screenDiag[DiagDebug]).toBool();
    m_screenDiag[DiagWarning]   = Setting->value("ScreenWarning",m_screenDiag[DiagWarning]).toBool();
    m_screenDiag[DiagProtobuf]  = Setting->value("ScreenProto"  ,m_screenDiag[DiagProtobuf]).toBool();

    m_logDiag[DiagInfo]         = Setting->value("LogInfo"      ,m_logDiag[DiagInfo]).toBool();
    m_logDiag[DiagError]        = Setting->value("LogError"     ,m_logDiag[DiagError]).toBool();
    m_logDiag[DiagTrace]        = Setting->value("LogTrace"     ,m_logDiag[DiagTrace]).toBool();
    m_logDiag[DiagDebug]        = Setting->value("LogDebug"     ,m_logDiag[DiagDebug]).toBool();
    m_logDiag[DiagWarning]      = Setting->value("LogWarning"   ,m_logDiag[DiagWarning]).toBool();
    m_logDiag[DiagProtobuf]     = Setting->value("LogProto"     ,m_logDiag[DiagProtobuf]).toBool();

    m_filePath     = Setting->value("FilePath",m_filePath).toString();
    m_fileName     = Setting->value("FileName",m_fileName).toString();
}

void Logger::saveSettings    (QSettings *Setting)
{
    Setting->setValue("ScreenInfo"      ,m_screenDiag[DiagInfo]);
    Setting->setValue("ScreenError"     ,m_screenDiag[DiagError]);
    Setting->setValue("ScreenTrace"     ,m_screenDiag[DiagTrace]);
    Setting->setValue("ScreenDebug"     ,m_screenDiag[DiagDebug]);
    Setting->setValue("ScreenWarning"   ,m_screenDiag[DiagWarning]);
    Setting->setValue("ScreenProto"     ,m_screenDiag[DiagProtobuf]);

    Setting->setValue("LogInfo"         ,m_logDiag[DiagInfo]);
    Setting->setValue("LogError"        ,m_logDiag[DiagError]);
    Setting->setValue("LogTrace"        ,m_logDiag[DiagTrace]);
    Setting->setValue("LogDebug"        ,m_logDiag[DiagDebug]);
    Setting->setValue("LogWarning"      ,m_logDiag[DiagWarning]);
    Setting->setValue("LogProto"        ,m_logDiag[DiagProtobuf]);

    Setting->setValue("FilePath",m_filePath);
    Setting->setValue("FileName",m_fileName);
}

bool Logger::start(){

    m_active=true;
    return true;
}

bool Logger::stop(){
    closeLogFile();
    m_active=false;
    return true;
}

bool Logger::openLogFile(QString filePath)
{
    m_file.setFileName(filePath);
    if (!m_file.open(QIODevice::WriteOnly | QIODevice::Text)){
        return false;
    }

    INFO << "Log file succesfully opened: "<<m_filePath<<"/"<<m_fileName;

    return true;
}

bool Logger::closeLogFile(void)
{
    if  (m_file.isOpen()){
        m_file.close();
        return true;
    }else{
        ERRO <<"File is not open. Failed to write to: "<<m_filePath<<"/"<<m_fileName;
        return false;
    }
}

void Logger::protobufPrintSlot(google::protobuf::Message *msg){
    QString text = QString::fromStdString(msg->DebugString());
    emit diagSignal(DiagProtobuf, text);
}

bool Logger::sendMessageToLog(QString message)
{
    if (!m_active) return false;
    m_mutex.lock();

    QDateTime currentDateTime=QDateTime::currentDateTime();
    if  (m_file.isOpen()){
        if (m_lastLogDate!=currentDateTime.date()){
            closeLogFile();
            openLogFile(m_filePath+"/"+m_fileName.split(".",QString::SkipEmptyParts).at(0)+"_"+currentDateTime.toString("(hh_mm_ss)-[dd_MM_yyyy]")+".txt");
        }
    }else{
        openLogFile(m_filePath+"/"+m_fileName.split(".",QString::SkipEmptyParts).at(0)+"_"+currentDateTime.toString("(hh_mm_ss)-[dd_MM_yyyy]")+".txt");
    }
    m_lastLogDate=currentDateTime.date();

    if  (m_file.isOpen()){
        QTextStream out(&m_file);
        out << message<< "\n";
        m_mutex.unlock();
        return true;
    }else{
        m_mutex.unlock();
        return false;
    }
}

QString Logger::getTimeStamp()
{
    QString timestamp;
    // set timestamp
    return timestamp;

}

void Logger::diagSlot(int Type, QString Text){
    diagSlot((DiagType)Type, Text);
}

void Logger::diagSlot(DiagType Type, QString Text)
{
    QString Timestamp= tr("%1").arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"));
    QString Class = "Unknown";
    if(sender()!=NULL){
        Class = tr("%1").arg("["+QString(sender()->metaObject()->className())+"]",-50);
    }
    captoglove_v1::LogMsg msg;
    QString TextToPrint;

    switch (Type) {
        case DiagInfo:
            msg.set_type(captoglove_v1::LogType_Info);
            TextToPrint = Timestamp+" [INFO]  "+Class+ Text;
            if(m_screenDiag[DiagInfo]) std::cout<<TextToPrint.toStdString()<<"\n";
            if(m_logDiag[DiagInfo]) sendMessageToLog(TextToPrint);
            break;
        case DiagError:
            msg.set_type(captoglove_v1::LogType_Error);
            TextToPrint = Timestamp+" [ERROR] "+Class+ Text;
            if(m_screenDiag[DiagError]) std::cout<<TextToPrint.toStdString()<<"\n";
            if(m_logDiag[DiagError]) sendMessageToLog(TextToPrint);
            break;
        case DiagTrace:
            msg.set_type(captoglove_v1::LogType_Debug);
            TextToPrint = Timestamp+" [TRACE] "+Class+ Text;
            if(m_screenDiag[DiagTrace]) std::cout<<TextToPrint.toStdString()<<"\n";
            if(m_logDiag[DiagTrace]) sendMessageToLog(TextToPrint);
            break;
        case DiagDebug:
            msg.set_type(captoglove_v1::LogType_Debug);
            TextToPrint = Timestamp+" [DEBUG] "+Class+ Text;
            if(m_screenDiag[DiagDebug]) std::cout<<TextToPrint.toStdString()<<"\n";
            if(m_logDiag[DiagDebug]) sendMessageToLog(TextToPrint);
            break;
        case DiagWarning:
            msg.set_type(captoglove_v1::LogType_Warning);
            TextToPrint = Timestamp+" [WARN]  "+Class+ Text;
            if(m_screenDiag[DiagWarning]) std::cout<<TextToPrint.toStdString()<<"\n";
            if(m_logDiag[DiagWarning]) sendMessageToLog(TextToPrint);
            break;
        case DiagProtobuf:
            msg.set_type(captoglove_v1::LogType_Debug);
            TextToPrint = Timestamp+" [PROTO] "+Class+ Text;
            if(m_screenDiag[DiagProtobuf]) std::cout<<TextToPrint.toStdString()<<"\n";
            if(m_logDiag[DiagProtobuf]) sendMessageToLog(TextToPrint);
            break;
    default:
        break;
    }

    msg.set_message(TextToPrint.toStdString());
    emit logSignal(msg);
    emit diagSignal(Type, TextToPrint);
}

QString Logger::filePath() const
{
    return m_filePath;
}

void Logger::setFilePath(const QString &FilePath)
{
    m_filePath = FilePath;
}

QString Logger::fileName() const
{
    return m_fileName;
}

void Logger::setFileName(const QString &FileName)
{
    m_fileName = FileName;
}

