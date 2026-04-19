#include <Arduino.h>

class xbee {
private:
    const double GOAL_LEFT[2] = {0,52.5};
    const double GOAL_RIGHT[2] = {210,52.5};
    double targetGoal[2];

    bool xbeeHasValidPosition = false;
    bool mazeStarted = false;
    bool mazeFinished = false;

    // latest position from Xbee
    double currPosition[2];
    
    char rxBuffer[128];
    int rxIndex;
    unsigned long lastRxTime;
    #define RX_TIMEOUT_MS 5
    #define ROBOT_ID 'D'

    void updateTargetGoal(bool attackRight);

    int extractDigits(const char* buf, int len, int &pos, int numDigits);
    bool parseBroadcast(const char* buf);
    void processXBeeMessage();
    

public:
    xbee();

    bool gameStarted();
    void updateXBeePosition();
    bool inBox(int x, int y, int xmin, int xmax, int ymin, int ymax);
    const double* currentPosition();
    const double* opponentPosition();
    const double* LeftGoalPosition();
    const double* rightGoalPosition();
};

