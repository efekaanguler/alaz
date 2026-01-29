#include <mission_control/mode_base.hpp>

class RunMode : public ModeBase {
public:
    unsigned int execute() override;
};