#pragma once

#include "Arduino.h"
#include "PointXY.h"

#define MAXPOINTS 200

class TimeFunction {
    public:
    unsigned long t = 0;
    unsigned long tStart = millis();
    PointXY p[MAXPOINTS];
    int lengthPoints = 1;
    bool _isFinished = false;
    int indexPoint = 0;
    double m, b;

    void resetTime()
    {
        tStart = millis();
        indexPoint = 0;
        _isFinished = false;
    }

    double getValue()
    {
        t = millis() - tStart;

        if (t > p[indexPoint].x) {
            indexPoint += 1;
            if (indexPoint >= lengthPoints) {
                _isFinished = true;
                indexPoint = lengthPoints - 1;
                return 0;
            }
            m = (p[indexPoint].y - p[indexPoint - 1].y) / (p[indexPoint].x - p[indexPoint - 1].x);
            b = p[indexPoint].y - m * p[indexPoint].x;
        }

        return m * t + b;
    }

    void setPoints(PointXY points[], int length)
    {
        for (int i = 0; i < length; i++) {
            p[i] = points[i];
            Serial.print("x_");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(p[i].x);
            Serial.print(" y_");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(p[i].y);
        }
        lengthPoints = length;
    }

    bool isFinished()
    {
        return _isFinished;
    }
};