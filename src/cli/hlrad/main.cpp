#include "../../common/log.h"
#include "../tools/rad_tool.h"

int main(int argc, char **argv)
{
    logging::init_console();
    return tools::run_rad_tool(argc, argv);
}
