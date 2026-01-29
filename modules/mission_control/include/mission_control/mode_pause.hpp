#include <mission_control/mode_base.hpp>

class PauseMode : public ModeBase {
public:
    unsigned int execute() override;
};