#include <mission_control/mode_base.hpp>

class ParkMode : public ModeBase {
public:
    unsigned int execute() override;
};