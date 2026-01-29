#include <mission_control/mode_base.hpp>

class StartMode : public ModeBase {
public:
    unsigned int execute() override;
};