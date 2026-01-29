#include <mission_control/mode_base.hpp>

class EmergencyMode : public ModeBase {
public:
    unsigned int execute() override;
};