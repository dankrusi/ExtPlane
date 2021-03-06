#include "datadataref.h"

DataDataRef::DataDataRef(QObject *parent, QString name, XPLMDataRef ref) : DataRef(parent, name, ref)
{
    // Init
    _typeString = "b";
    _type = xplmType_Data;
    _length = XPLMGetDatab(_ref, NULL, 0, 0);
    _value = QByteArray(_length, 0);
    _newValue = QByteArray(_length, 0); // Init already here for perf reasons.
    qDebug() << Q_FUNC_INFO << "Inited data dataref with a length of =" << _length;
}

QByteArray &DataDataRef::value() {
    return _value;
}

void DataDataRef::updateValue() {
    // Read and verify data from XPLM
    int valuesCopied = XPLMGetDatab(_ref, _newValue.data(), 0, _length);
    Q_ASSERT(valuesCopied == _length);

    if (_newValue != _value) {
        _value = _newValue;
        emit changed(this);
    }
}

void DataDataRef::setValue(QByteArray &newValue) {
    //TODO: @dankrusi: finish this implementation and test
    qFatal("Writing of Data DataRefs is not yet supported");
    /*
    // Limit number of values to write to ref length or number of given values
    int numberOfValuesToWrite = qMin(_length, values.size());

    // Convert values to float and copy to _valueArray
    for(int i=0;i<numberOfValuesToWrite;i++) {
        bool ok = true;
        float value = values[i].toFloat(&ok);
        if(!ok) {
            qDebug() << Q_FUNC_INFO << "Invalid value " << values[i] << "in array";
            return;
        }
        _valueArray[i]=value;
    }
    XPLMSetDatavf(_ref, _valueArray, 0, numberOfValuesToWrite);
    */
}

QString DataDataRef::valueString() {
    return QString(_value).toUtf8().toBase64();
}

void DataDataRef::setValue(QString &newValue) {
    QByteArray valueBA = newValue.toUtf8();
    setValue(valueBA);
}
