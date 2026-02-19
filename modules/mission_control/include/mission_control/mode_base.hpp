#ifndef MODE_BASE_HPP
#define MODE_BASE_HPP

enum MODE: unsigned int {
    MODE_START=0,
    MODE_RUN=1,
    MODE_PAUSE=2,
    MODE_PARK=3,
    MODE_EMERGENCY=4
};

class ModeBase {
public:
    virtual ~ModeBase() {}

    virtual unsigned int execute() = 0; 
};

#endif